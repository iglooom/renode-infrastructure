//
// Copyright (c) 2026
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;
using System.Threading.Tasks;
using System.Text;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Memory;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Video
{
    // MPC5645S/PXD20 DCU3 subset exercised by the tested MPC5645S firmware.
    //
    // The firmware programs timing and layer registers, unmasks INT_STATUS bits
    // 17 (DMA_TRANS_FINISH, numeric 0x4000) and 28 (VS_BLANK, numeric 0x8),
    // then waits for the source-184 ISR. Frame events are scheduled in Renode
    // virtual time; they are never raised synchronously from an MMIO write.
    // Layer/CLUT state becomes exported scanout state only on a later frame
    // boundary, keeping the firmware's RAM shadow distinct from committed DCU
    // hardware state.
    public sealed class MPC5645S_DCU : IDoubleWordPeripheral, IWordPeripheral,
        IBytePeripheral, IKnownSize, IHasFrequency
    {
        public MPC5645S_DCU(IMachine machine, ulong frameFrequency = DefaultFrameFrequency)
        {
            if(frameFrequency == 0)
            {
                throw new ArgumentOutOfRangeException(nameof(frameFrequency));
            }
            this.machine = machine;
            systemBus = machine.GetSystemBus(this);
            registers = new uint[SizeValue / 4];
            committedRegisters = new uint[registers.Length];
            frameTimer = new LimitTimer(machine.ClockSource, frameFrequency, this, nameof(frameTimer),
                limit: 1, direction: Direction.Descending, enabled: false,
                workMode: WorkMode.OneShot, eventEnabled: true, autoUpdate: false);
            frameTimer.LimitReached += OnFrameBoundary;
            IRQ = new GPIO();
            Reset();
        }

        public void Reset()
        {
            lock(sync)
            {
                Array.Clear(registers, 0, registers.Length);
                Array.Clear(committedRegisters, 0, committedRegisters.Length);
                frameTimer.Reset();
                frameTimer.AutoUpdate = false;
                frameTimer.EventEnabled = true;
                frameTimer.Limit = 1;
                frameTimer.Value = 1;
                frameTimer.Enabled = false;
                frameEventsEnabled = true;
                committed = false;
                FrameEventCount = 0;
                CommittedFrameCount = 0;
                LayerRegisterWriteCount = 0;
                ClutRegisterWriteCount = 0;
                IRQ.Unset();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            return ReadSized(offset, 4);
        }

        public ushort ReadWord(long offset)
        {
            return (ushort)ReadSized(offset, 2);
        }

        public byte ReadByte(long offset)
        {
            return (byte)ReadSized(offset, 1);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            WriteSized(offset, 4, value);
        }

        public void WriteWord(long offset, ushort value)
        {
            WriteSized(offset, 2, value);
        }

        public void WriteByte(long offset, byte value)
        {
            WriteSized(offset, 1, value);
        }

        public ulong Frequency
        {
            get => frameTimer.Frequency;
            set
            {
                if(value == 0)
                {
                    throw new ArgumentOutOfRangeException(nameof(value));
                }
                lock(sync)
                {
                    frameTimer.Frequency = value;
                    if(ShouldScheduleFrames)
                    {
                        RestartFrameTimer();
                    }
                }
            }
        }

        // Host-only matched-control switch. It does not alter guest-visible
        // registers and defaults to normal scheduled frame generation.
        public bool FrameEventsEnabled
        {
            get => frameEventsEnabled;
            set
            {
                lock(sync)
                {
                    frameEventsEnabled = value;
                    UpdateFrameTimer();
                }
            }
        }

        public long Size => SizeValue;

        public GPIO IRQ { get; }

        public ulong FrameEventCount { get; private set; }

        public ulong CommittedFrameCount { get; private set; }

        public ulong LayerRegisterWriteCount { get; private set; }

        public ulong ClutRegisterWriteCount { get; private set; }

        public uint InterruptStatus => registers[InterruptStatusOffset / 4];

        public uint InterruptMask => registers[InterruptMaskOffset / 4];

        public bool IsEnabled => (registers[ModeOffset / 4] & ModeFieldMask) != 0;

        public bool HasCommittedState => committed;

        public uint GetCommittedLayerWord(int layer, int word)
        {
            lock(sync)
            {
                if(layer < 0 || layer >= LayerCount || word < 0 || word >= LayerWordCount)
                {
                    return 0;
                }
                return committedRegisters[(layer * LayerStride + word * 4) / 4];
            }
        }

        public uint GetCommittedAuxWord(int layer, int word)
        {
            lock(sync)
            {
                if(layer < 0 || layer >= LayerCount || word < 0 || word >= 2)
                {
                    return 0;
                }
                return committedRegisters[(AuxiliaryLayerBaseOffset + layer * 8 + word * 4) / 4];
            }
        }

        public uint GetCommittedClutEntry(int index)
        {
            lock(sync)
            {
                if(index < 0 || index >= ClutEntryCount)
                {
                    return 0;
                }
                return committedRegisters[(ClutBaseOffset / 4) + index];
            }
        }

        public bool ExportCommittedState(string directory)
        {
            lock(sync)
            {
                if(!committed)
                {
                    this.Log(LogLevel.Warning, "Refusing to export DCU state before a hardware frame boundary");
                    return false;
                }
                try
                {
                    var exportDirectory = Path.GetFullPath(directory);
                    Directory.CreateDirectory(exportDirectory);
                    ExportClut(exportDirectory);
                    var state = BuildStateJson(exportDirectory);
                    File.WriteAllText(Path.Combine(exportDirectory, StateFileName), state, Encoding.UTF8);
                    return true;
                }
                catch(Exception exception)
                {
                    this.Log(LogLevel.Error, "DCU committed-state export failed: {0}", exception.Message);
                    return false;
                }
            }
        }

        // Live display output: at most every `minimumIntervalMicroseconds` of
        // virtual time, the committed scanout state is composed to RGBA and
        // published to a memory-mapped file. Header (little-endian):
        //   0 u32 magic 'DCUF', 4 u32 sequence (odd while writing),
        //   8 u32 width, 12 u32 height, 16 u64 virtual time ns,
        //   24 u64 committed frame count, 32 u64 published frame count,
        //   40 u32 last compose error flag; pixels (RGBA) start at 64.
        // Layer buffers are copied on the CPU thread; composition runs on a
        // worker thread so the guest is not stalled by it.
        public bool EnableSharedFrameOutput(string path, ulong minimumIntervalMicroseconds = 33333)
        {
            lock(sync)
            {
                DisableSharedFrameOutput();
                var fullPath = Path.GetFullPath(path);
                using(var stream = new FileStream(fullPath, FileMode.Create, FileAccess.ReadWrite, FileShare.ReadWrite))
                {
                    stream.SetLength(SharedHeaderBytes + SharedMaximumPixels * 4L);
                }
                sharedFile = MemoryMappedFile.CreateFromFile(fullPath, FileMode.Open, null, 0, MemoryMappedFileAccess.ReadWrite);
                sharedView = sharedFile.CreateViewAccessor(0, 0, MemoryMappedFileAccess.ReadWrite);
                sharedView.Write(0, SharedMagic);
                sharedView.Write(4, 0u);
                publishIntervalTicks = minimumIntervalMicroseconds * (TimeInterval.TicksPerSecond / 1000000UL);
                lastPublishTicks = 0;
                hasPublished = false;
                PublishedFrameCount = 0;
                return true;
            }
        }

        public void DisableSharedFrameOutput()
        {
            lock(sync)
            {
                composeTask?.Wait(1000);
                composeTask = null;
                sharedView?.Dispose();
                sharedFile?.Dispose();
                sharedView = null;
                sharedFile = null;
            }
        }

        // Writes the committed state composed by the same code as the live
        // output, as raw RGBA, for differential tests against the Python renderer.
        public bool ExportComposedRgba(string path)
        {
            lock(sync)
            {
                if(!committed)
                {
                    return false;
                }
                var snapshot = CaptureScanout();
                var rgba = Compose(snapshot, out var _, out var _);
                File.WriteAllBytes(Path.GetFullPath(path), rgba);
                return true;
            }
        }

        public ulong PublishedFrameCount { get; private set; }
        public string LastComposeError { get; private set; }

        private void MaybePublishFrame()
        {
            if(sharedView == null)
            {
                return;
            }
            var now = machine.ElapsedVirtualTime.TimeElapsed.Ticks;
            if(hasPublished && now - lastPublishTicks < publishIntervalTicks)
            {
                return;
            }
            if(composeTask != null && !composeTask.IsCompleted)
            {
                // Previous composition still running: skip, never block the guest.
                return;
            }
            lastPublishTicks = now;
            hasPublished = true;
            ScanoutSnapshot snapshot;
            try
            {
                snapshot = CaptureScanout();
            }
            catch(Exception exception)
            {
                ReportComposeError(exception);
                return;
            }
            snapshot.VirtualTimeTicks = now;
            snapshot.CommittedFrameCount = CommittedFrameCount;
            var view = sharedView;
            composeTask = Task.Run(() => ComposeAndPublish(snapshot, view));
        }

        private void ComposeAndPublish(ScanoutSnapshot snapshot, MemoryMappedViewAccessor view)
        {
            try
            {
                var rgba = Compose(snapshot, out var width, out var height);
                if(width * height > SharedMaximumPixels)
                {
                    throw new InvalidOperationException($"display {width}x{height} exceeds shared buffer");
                }
                lock(publishSync)
                {
                    var sequence = view.ReadUInt32(4);
                    view.Write(4, sequence | 1u);
                    Thread.MemoryBarrier();
                    view.Write(8, (uint)width);
                    view.Write(12, (uint)height);
                    view.Write(16, snapshot.VirtualTimeTicks * (1000000000UL / TimeInterval.TicksPerSecond));
                    view.Write(24, snapshot.CommittedFrameCount);
                    view.Write(32, PublishedFrameCount + 1);
                    view.Write(40, 0u);
                    view.WriteArray(SharedHeaderBytes, rgba, 0, rgba.Length);
                    Thread.MemoryBarrier();
                    view.Write(4, (sequence | 1u) + 1u);
                    PublishedFrameCount++;
                }
            }
            catch(Exception exception)
            {
                ReportComposeError(exception);
            }
        }

        private void ReportComposeError(Exception exception)
        {
            if(LastComposeError != exception.Message)
            {
                this.Log(LogLevel.Warning, "DCU frame composition failed: {0}", exception.Message);
            }
            LastComposeError = exception.Message;
        }

        private sealed class ScanoutSnapshot
        {
            public uint[] Registers;
            public byte[][] LayerData = new byte[LayerCount][];
            public ulong VirtualTimeTicks;
            public ulong CommittedFrameCount;
        }

        // Copies committed registers and the bytes of each enabled layer buffer.
        private ScanoutSnapshot CaptureScanout()
        {
            var snapshot = new ScanoutSnapshot { Registers = (uint[])committedRegisters.Clone() };
            for(var layer = 0; layer < LayerCount; layer++)
            {
                var words = LayerWords(snapshot.Registers, layer);
                var control = words[3];
                var width = (int)(words[0] & 0x7FF);
                var height = (int)((words[0] >> 16) & 0x7FF);
                if((control & LayerEnableMask) == 0 || width == 0 || height == 0 || words[2] == 0)
                {
                    continue;
                }
                if((control & ClutAsTileSourceMask) != 0)
                {
                    throw new InvalidOperationException($"layer {layer} uses CLUT-as-tile source");
                }
                var tileWidth = width;
                var tileHeight = height;
                if((control & TiledLayerMask) != 0)
                {
                    tileWidth = (int)(words[6] & 0x7F) * 16;
                    tileHeight = (int)((words[6] >> 16) & 0x7FF);
                }
                var length = PixelBufferLength((int)((control >> 16) & 0xF), tileWidth, tileHeight);
                if(length <= 0 || length > MaximumExportBufferBytes)
                {
                    throw new InvalidOperationException($"layer {layer} has unsupported geometry/format");
                }
                if(!IsMappedMemoryRange(words[2], (uint)length))
                {
                    throw new InvalidOperationException($"layer {layer} framebuffer 0x{words[2]:X8} is not mapped memory");
                }
                snapshot.LayerData[layer] = systemBus.ReadBytes(words[2], length, onlyMemory: true, context: this);
            }
            return snapshot;
        }

        private static uint[] LayerWords(uint[] regs, int layer)
        {
            var words = new uint[LayerWordCount];
            for(var word = 0; word < LayerWordCount; word++)
            {
                words[word] = regs[(layer * LayerStride + word * 4) / 4];
            }
            return words;
        }

        // Port of tools/render_dcu_frame.py; keep both in sync (tested for
        // byte equality by tests/run_dcu_compose_tests.py).
        private static byte[] Compose(ScanoutSnapshot snapshot, out int width, out int height)
        {
            var regs = snapshot.Registers;
            var displaySize = regs[DisplaySizeOffset / 4];
            width = (int)(displaySize & 0x7F) * 16;
            height = (int)((displaySize >> 16) & 0x7FF);
            if(width <= 0 || height <= 0 || width > 2048 || height > 2048)
            {
                throw new InvalidOperationException($"invalid display size {width}x{height}");
            }
            var background = regs[BackgroundOffset / 4];
            var frame = new byte[width * height * 4];
            for(var i = 0; i < frame.Length; i += 4)
            {
                frame[i] = (byte)(background >> 16);
                frame[i + 1] = (byte)(background >> 8);
                frame[i + 2] = (byte)background;
                frame[i + 3] = 255;
            }
            var clut = new uint[ClutEntryCount];
            Array.Copy(regs, ClutBaseOffset / 4, clut, 0, ClutEntryCount);

            for(var layer = LayerCount - 1; layer >= 0; layer--)
            {
                var data = snapshot.LayerData[layer];
                if(data == null)
                {
                    continue;
                }
                var words = LayerWords(regs, layer);
                var layerWidth = (int)(words[0] & 0x7FF);
                var layerHeight = (int)((words[0] >> 16) & 0x7FF);
                var layerX = Signed12(words[1]);
                var layerY = Signed12(words[1] >> 16);
                var control = words[3];
                var format = (int)((control >> 16) & 0xF);
                var paletteOffset = (int)((control >> 4) & 0x7FF);
                if((control & 0x8000) != 0 || format == 14 || format == 15)
                {
                    throw new InvalidOperationException($"layer {layer} uses unsupported RLE/YCbCr format {format}");
                }
                var tileWidth = layerWidth;
                var tileHeight = layerHeight;
                if((control & TiledLayerMask) != 0)
                {
                    tileWidth = (int)(words[6] & 0x7F) * 16;
                    tileHeight = (int)((words[6] >> 16) & 0x7FF);
                    if(tileWidth <= 0 || tileHeight <= 0)
                    {
                        throw new InvalidOperationException($"layer {layer} has invalid tile geometry");
                    }
                }
                var foreground = regs[(AuxiliaryLayerBaseOffset + layer * 8) / 4];
                var layerBackground = regs[(AuxiliaryLayerBaseOffset + layer * 8 + 4) / 4];
                var alphaBlend = control & 3;
                var transparencyValue = (int)((control >> 20) & 0xFF);
                var chromaMax = words[4];
                var chromaMin = words[5];
                var chromaEnabled = (control & 4) != 0;
                var argb = format <= 3 || format == 6 || format >= 11;
                var transparency = format == 7 || format == 8;

                for(var yy = 0; yy < layerHeight; yy++)
                {
                    var destinationY = layerY + yy;
                    if(destinationY < 0 || destinationY >= height)
                    {
                        continue;
                    }
                    var py = yy % tileHeight;
                    for(var xx = 0; xx < layerWidth; xx++)
                    {
                        var destinationX = layerX + xx;
                        if(destinationX < 0 || destinationX >= width)
                        {
                            continue;
                        }
                        var px = xx % tileWidth;
                        int red = 0, green = 0, blue = 0, alpha = 255;
                        int value, offset;
                        switch(format)
                        {
                            case 0:
                                value = Byte(data, py * ((tileWidth + 7) / 8) + px / 8, layer);
                                Clut(clut, paletteOffset + ((value >> (px & 7)) & 1), out red, out green, out blue, out alpha);
                                break;
                            case 1:
                                value = Byte(data, py * ((tileWidth + 3) / 4) + px / 4, layer);
                                Clut(clut, paletteOffset + ((value >> (2 * (px & 3))) & 3), out red, out green, out blue, out alpha);
                                break;
                            case 2:
                                value = Byte(data, py * ((tileWidth + 1) / 2) + px / 2, layer);
                                Clut(clut, paletteOffset + ((value >> (4 * (px & 1))) & 0xF), out red, out green, out blue, out alpha);
                                break;
                            case 3:
                                value = Byte(data, py * tileWidth + px, layer);
                                Clut(clut, paletteOffset + value, out red, out green, out blue, out alpha);
                                break;
                            case 4:
                                value = Word(data, 2 * (py * tileWidth + px), layer);
                                red = ((value >> 11) & 0x1F) * 255 / 31;
                                green = ((value >> 5) & 0x3F) * 255 / 63;
                                blue = (value & 0x1F) * 255 / 31;
                                break;
                            case 5:
                                offset = 3 * (py * tileWidth + px);
                                blue = Byte(data, offset, layer);
                                green = Byte(data, offset + 1, layer);
                                red = Byte(data, offset + 2, layer);
                                break;
                            case 6:
                                offset = 4 * (py * tileWidth + px);
                                blue = Byte(data, offset, layer);
                                green = Byte(data, offset + 1, layer);
                                red = Byte(data, offset + 2, layer);
                                alpha = Byte(data, offset + 3, layer);
                                break;
                            case 11:
                                value = Word(data, 2 * (py * tileWidth + px), layer);
                                alpha = (value & 0x8000) != 0 ? 255 : 0;
                                red = ((value >> 10) & 0x1F) * 255 / 31;
                                green = ((value >> 5) & 0x1F) * 255 / 31;
                                blue = (value & 0x1F) * 255 / 31;
                                break;
                            case 12:
                                value = Word(data, 2 * (py * tileWidth + px), layer);
                                alpha = ((value >> 12) & 0xF) * 17;
                                red = ((value >> 8) & 0xF) * 17;
                                green = ((value >> 4) & 0xF) * 17;
                                blue = (value & 0xF) * 17;
                                break;
                            case 13:
                                offset = 2 * (py * tileWidth + px);
                                alpha = Byte(data, offset, layer);
                                Clut(clut, paletteOffset + Byte(data, offset + 1, layer), out red, out green, out blue, out var _);
                                break;
                            case 7:
                            case 8:
                            case 9:
                            case 10:
                                if(format == 7 || format == 9)
                                {
                                    value = Byte(data, py * ((tileWidth + 1) / 2) + px / 2, layer);
                                    alpha = ((value >> (4 * (px & 1))) & 0xF) << 4;
                                }
                                else
                                {
                                    alpha = Byte(data, py * tileWidth + px, layer);
                                }
                                if(!transparency)
                                {
                                    var o = 4 * (destinationY * width + destinationX);
                                    var delta = alpha < 128 ? alpha : alpha - 256;
                                    frame[o] = Clamp(frame[o] + delta);
                                    frame[o + 1] = Clamp(frame[o + 1] + delta);
                                    frame[o + 2] = Clamp(frame[o + 2] + delta);
                                    continue;
                                }
                                red = (int)((foreground >> 16) & 0xFF);
                                green = (int)((foreground >> 8) & 0xFF);
                                blue = (int)(foreground & 0xFF);
                                if((control & 7) != 6)
                                {
                                    red = (red * alpha + (int)((layerBackground >> 16) & 0xFF) * (255 - alpha)) / 255;
                                    green = (green * alpha + (int)((layerBackground >> 8) & 0xFF) * (255 - alpha)) / 255;
                                    blue = (blue * alpha + (int)(layerBackground & 0xFF) * (255 - alpha)) / 255;
                                    alpha = 255;
                                }
                                break;
                            default:
                                throw new InvalidOperationException($"layer {layer} uses unsupported pixel format {format}");
                        }

                        var selected = chromaEnabled
                            && ((chromaMin >> 16) & 0xFF) <= red && red <= ((chromaMax >> 16) & 0xFF)
                            && ((chromaMin >> 8) & 0xFF) <= green && green <= ((chromaMax >> 8) & 0xFF)
                            && (chromaMin & 0xFF) <= blue && blue <= (chromaMax & 0xFF);
                        if(selected && alphaBlend != 1)
                        {
                            continue;
                        }
                        if(alphaBlend == 0 || alphaBlend == 3)
                        {
                            alpha = 255;
                        }
                        else if(alphaBlend == 1)
                        {
                            if(chromaEnabled && !selected)
                            {
                                alpha = 255;
                            }
                            else if(!argb)
                            {
                                alpha = transparencyValue;
                            }
                        }
                        else
                        {
                            alpha = (argb || transparency) ? alpha * transparencyValue / 255 : transparencyValue;
                        }

                        var p = 4 * (destinationY * width + destinationX);
                        if(alpha >= 255)
                        {
                            frame[p] = (byte)red;
                            frame[p + 1] = (byte)green;
                            frame[p + 2] = (byte)blue;
                            frame[p + 3] = 255;
                        }
                        else
                        {
                            var inverse = 255 - alpha;
                            frame[p] = (byte)((red * alpha + frame[p] * inverse) / 255);
                            frame[p + 1] = (byte)((green * alpha + frame[p + 1] * inverse) / 255);
                            frame[p + 2] = (byte)((blue * alpha + frame[p + 2] * inverse) / 255);
                            frame[p + 3] = 255;
                        }
                    }
                }
            }
            return frame;
        }

        private static int Signed12(uint value)
        {
            var v = (int)(value & 0xFFF);
            return (v & 0x800) != 0 ? v - 0x1000 : v;
        }

        private static byte Clamp(int value)
        {
            return (byte)(value < 0 ? 0 : value > 255 ? 255 : value);
        }

        private static int Byte(byte[] data, int offset, int layer)
        {
            if(offset < 0 || offset >= data.Length)
            {
                throw new InvalidOperationException($"layer {layer} pixel offset {offset} exceeds {data.Length} bytes");
            }
            return data[offset];
        }

        private static int Word(byte[] data, int offset, int layer)
        {
            if(offset < 0 || offset + 2 > data.Length)
            {
                throw new InvalidOperationException($"layer {layer} pixel word offset {offset} exceeds {data.Length} bytes");
            }
            return (data[offset] << 8) | data[offset + 1];
        }

        private static void Clut(uint[] clut, int index, out int red, out int green, out int blue, out int alpha)
        {
            var value = clut[index & 0x7FF];
            red = (int)((value >> 16) & 0xFF);
            green = (int)((value >> 8) & 0xFF);
            blue = (int)(value & 0xFF);
            alpha = (int)((value >> 24) & 0xFF);
        }

        private uint ReadSized(long offset, int size)
        {
            if(!IsValidAccess(offset, size))
            {
                this.Log(LogLevel.Warning, "Unsupported DCU read offset/size: 0x{0:X}, {1}", offset, size);
                return 0;
            }
            SynchronizeCurrentCpuTime();
            lock(sync)
            {
                var alignedOffset = offset & ~0x3L;
                var value = registers[alignedOffset / 4];
                var lane = (int)(offset & 0x3L);
                var shift = (4 - lane - size) * 8;
                var mask = size == 1 ? 0xFFu : size == 2 ? 0xFFFFu : uint.MaxValue;
                return (value >> shift) & mask;
            }
        }

        private void WriteSized(long offset, int size, uint value)
        {
            if(!IsValidAccess(offset, size))
            {
                this.Log(LogLevel.Warning, "Unsupported DCU write offset/size: 0x{0:X}, {1}", offset, size);
                return;
            }
            lock(sync)
            {
                var alignedOffset = offset & ~0x3L;
                var lane = (int)(offset & 0x3L);
                var shift = (4 - lane - size) * 8;
                var laneMask = size == 1 ? 0xFFu : size == 2 ? 0xFFFFu : uint.MaxValue;
                var mask = laneMask << shift;
                var positionedValue = (value & laneMask) << shift;
                WriteRegister(alignedOffset, positionedValue, mask);
            }
        }

        private void WriteRegister(long offset, uint value, uint mask)
        {
            var index = offset / 4;
            var oldValue = registers[index];
            if(offset == InterruptStatusOffset)
            {
                // W1C uses the OLD status and only the written, positioned ones.
                // The OEM ISR clears 0x4000, re-reads, then clears 0x0008.
                registers[index] = oldValue & ~(value & mask);
                UpdateInterrupt();
                return;
            }

            registers[index] = (oldValue & ~mask) | (value & mask);
            if(offset >= 0 && offset < LayerCount * LayerStride)
            {
                LayerRegisterWriteCount++;
            }
            else if(offset >= ClutBaseOffset && offset < ClutBaseOffset + ClutEntryCount * 4)
            {
                ClutRegisterWriteCount++;
            }

            if(offset == ModeOffset)
            {
                UpdateFrameTimer();
                UpdateInterrupt();
            }
            else if(offset == InterruptMaskOffset)
            {
                UpdateInterrupt();
            }
        }

        private void OnFrameBoundary()
        {
            lock(sync)
            {
                frameTimer.Enabled = false;
                if(!ShouldScheduleFrames)
                {
                    UpdateInterrupt();
                    return;
                }

                Array.Copy(registers, committedRegisters, registers.Length);
                committed = true;
                FrameEventCount++;
                CommittedFrameCount++;
                MaybePublishFrame();
                registers[InterruptStatusOffset / 4] |= FrameInterruptBits;
                UpdateInterrupt();
                RestartFrameTimer();
            }
        }

        private void UpdateFrameTimer()
        {
            if(ShouldScheduleFrames)
            {
                if(!frameTimer.Enabled)
                {
                    RestartFrameTimer();
                }
            }
            else
            {
                frameTimer.Enabled = false;
            }
        }

        private void RestartFrameTimer()
        {
            frameTimer.Enabled = false;
            frameTimer.Limit = 1;
            frameTimer.Value = 1;
            frameTimer.Enabled = true;
        }

        private void UpdateInterrupt()
        {
            var active = IsEnabled
                && (registers[InterruptStatusOffset / 4]
                    & ~registers[InterruptMaskOffset / 4]
                    & FrameInterruptBits) != 0;
            IRQ.Set(active);
        }

        private bool ShouldScheduleFrames => frameEventsEnabled && IsEnabled;

        private void ExportClut(string directory)
        {
            var bytes = new byte[ClutEntryCount * 4];
            for(var index = 0; index < ClutEntryCount; index++)
            {
                PutBigEndian(bytes, index * 4, committedRegisters[(ClutBaseOffset / 4) + index]);
            }
            File.WriteAllBytes(Path.Combine(directory, ClutFileName), bytes);
        }

        private string BuildStateJson(string directory)
        {
            var displaySize = committedRegisters[DisplaySizeOffset / 4];
            var width = (int)(displaySize & 0x7F) * 16;
            var height = (int)((displaySize >> 16) & 0x7FF);
            var builder = new StringBuilder();
            builder.AppendLine("{");
            AppendJsonProperty(builder, "committed", "true", true, 2);
            AppendJsonProperty(builder, "committed_frame_count", CommittedFrameCount.ToString(), true, 2);
            AppendJsonProperty(builder, "frame_event_count", FrameEventCount.ToString(), true, 2);
            AppendJsonProperty(builder, "frame_frequency", Frequency.ToString(), true, 2);
            AppendJsonProperty(builder, "mode", committedRegisters[ModeOffset / 4].ToString(), true, 2);
            AppendJsonProperty(builder, "background", committedRegisters[BackgroundOffset / 4].ToString(), true, 2);
            AppendJsonProperty(builder, "display_size", displaySize.ToString(), true, 2);
            AppendJsonProperty(builder, "width", width.ToString(), true, 2);
            AppendJsonProperty(builder, "height", height.ToString(), true, 2);
            AppendJsonProperty(builder, "interrupt_status_at_export", InterruptStatus.ToString(), true, 2);
            AppendJsonProperty(builder, "interrupt_mask_at_export", InterruptMask.ToString(), true, 2);
            AppendJsonProperty(builder, "layer_register_write_count", LayerRegisterWriteCount.ToString(), true, 2);
            AppendJsonProperty(builder, "clut_register_write_count", ClutRegisterWriteCount.ToString(), true, 2);
            AppendJsonProperty(builder, "clut_file", Quote(ClutFileName), true, 2);
            builder.AppendLine("  \"layers\": [");
            for(var layer = 0; layer < LayerCount; layer++)
            {
                var words = new uint[LayerWordCount];
                for(var word = 0; word < LayerWordCount; word++)
                {
                    words[word] = committedRegisters[(layer * LayerStride + word * 4) / 4];
                }
                var aux0 = committedRegisters[(AuxiliaryLayerBaseOffset + layer * 8) / 4];
                var aux1 = committedRegisters[(AuxiliaryLayerBaseOffset + layer * 8 + 4) / 4];
                var bufferFile = default(string);
                var bufferLength = 0;
                var tileWidth = (int)(words[0] & 0x7FF);
                var tileHeight = (int)((words[0] >> 16) & 0x7FF);
                var control = words[3];
                var enabled = (control & LayerEnableMask) != 0 && tileWidth != 0 && tileHeight != 0 && words[2] != 0;
                if(enabled && (control & TiledLayerMask) != 0)
                {
                    tileWidth = (int)(words[6] & 0x7F) * 16;
                    tileHeight = (int)((words[6] >> 16) & 0x7FF);
                }
                if(enabled && (control & ClutAsTileSourceMask) == 0)
                {
                    bufferLength = PixelBufferLength((int)((control >> 16) & 0xF), tileWidth, tileHeight);
                    if(bufferLength <= 0 || bufferLength > MaximumExportBufferBytes)
                    {
                        throw new InvalidOperationException($"layer {layer} has unsupported export geometry/format");
                    }
                    if(!IsMappedMemoryRange(words[2], (uint)bufferLength))
                    {
                        throw new InvalidOperationException($"layer {layer} framebuffer 0x{words[2]:X8}+0x{bufferLength:X} is not mapped memory");
                    }
                    var bytes = systemBus.ReadBytes(words[2], bufferLength, onlyMemory: true, context: this);
                    bufferFile = $"layer{layer:D2}_0x{words[2]:X8}.bin";
                    File.WriteAllBytes(Path.Combine(directory, bufferFile), bytes);
                }

                builder.AppendLine("    {");
                AppendJsonProperty(builder, "index", layer.ToString(), true, 6);
                builder.Append("      \"words\": [");
                for(var word = 0; word < LayerWordCount; word++)
                {
                    if(word != 0)
                    {
                        builder.Append(", ");
                    }
                    builder.Append(words[word]);
                }
                builder.AppendLine("],");
                builder.AppendLine($"      \"aux\": [{aux0}, {aux1}],");
                AppendJsonProperty(builder, "tile_width", tileWidth.ToString(), true, 6);
                AppendJsonProperty(builder, "tile_height", tileHeight.ToString(), bufferFile != null, 6);
                if(bufferFile != null)
                {
                    AppendJsonProperty(builder, "buffer_address", words[2].ToString(), true, 6);
                    AppendJsonProperty(builder, "buffer_length", bufferLength.ToString(), true, 6);
                    AppendJsonProperty(builder, "buffer_file", Quote(bufferFile), false, 6);
                }
                builder.Append("    }");
                builder.AppendLine(layer + 1 == LayerCount ? string.Empty : ",");
            }
            builder.AppendLine("  ]");
            builder.AppendLine("}");
            return builder.ToString();
        }

        private int PixelBufferLength(int format, int width, int height)
        {
            if(width <= 0 || height <= 0)
            {
                return 0;
            }
            long rowLength;
            switch(format)
            {
                case 0: rowLength = (width + 7L) / 8L; break;
                case 1: rowLength = (width + 3L) / 4L; break;
                case 2:
                case 7:
                case 9: rowLength = (width + 1L) / 2L; break;
                case 3:
                case 8:
                case 10: rowLength = width; break;
                case 4:
                case 11:
                case 12:
                case 13: rowLength = width * 2L; break;
                case 5: rowLength = width * 3L; break;
                case 6: rowLength = width * 4L; break;
                default: return 0;
            }
            var length = rowLength * height;
            return length > int.MaxValue ? 0 : (int)length;
        }

        private bool IsMappedMemoryRange(uint address, uint size)
        {
            if(size == 0 || (ulong)address + size > 0x100000000UL)
            {
                return false;
            }
            // Walk whole registrations instead of probing every byte.
            ulong current = address;
            ulong end = (ulong)address + size;
            while(current < end)
            {
                var registration = systemBus.WhatIsAt(current, this);
                if(registration == null || !(registration.Peripheral is MappedMemory))
                {
                    return false;
                }
                var range = registration.RegistrationPoint.Range;
                if(range.EndAddress < current)
                {
                    return false;
                }
                current = range.EndAddress + 1;
            }
            return true;
        }

        private bool IsValidAccess(long offset, int size)
        {
            var lane = (int)(offset & 0x3L);
            return offset >= 0 && offset + size <= SizeValue
                && (size == 1 || size == 2 || size == 4)
                && lane + size <= 4 && (size != 4 || lane == 0);
        }

        private void SynchronizeCurrentCpuTime()
        {
            if(systemBus.TryGetCurrentCPU(out var cpu))
            {
                cpu.SyncTime();
            }
        }

        private static void PutBigEndian(byte[] bytes, int offset, uint value)
        {
            bytes[offset] = (byte)(value >> 24);
            bytes[offset + 1] = (byte)(value >> 16);
            bytes[offset + 2] = (byte)(value >> 8);
            bytes[offset + 3] = (byte)value;
        }

        private static string Quote(string value)
        {
            return "\"" + value.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
        }

        private static void AppendJsonProperty(StringBuilder builder, string name, string value, bool comma, int indent)
        {
            builder.Append(' ', indent);
            builder.Append(Quote(name));
            builder.Append(": ");
            builder.Append(value);
            if(comma)
            {
                builder.Append(',');
            }
            builder.AppendLine();
        }

        private const int SizeValue = 0x4000;
        private const int LayerCount = 16;
        private const int LayerWordCount = 7;
        private const int LayerStride = 0x1C;
        private const int ClutEntryCount = 2048;
        private const int MaximumExportBufferBytes = 64 * 1024 * 1024;
        private const ulong DefaultFrameFrequency = 60;

        private const long ModeOffset = 0x1D0;
        private const long BackgroundOffset = 0x1D4;
        private const long DisplaySizeOffset = 0x1D8;
        private const long InterruptStatusOffset = 0x1EC;
        private const long InterruptMaskOffset = 0x1F0;
        private const int AuxiliaryLayerBaseOffset = 0x250;
        private const int ClutBaseOffset = 0x2000;

        private const uint ModeFieldMask = 0x3;
        private const uint FrameInterruptBits = 0x00004008;
        private const uint LayerEnableMask = 0x80000000;
        private const uint TiledLayerMask = 0x40000000;
        private const uint ClutAsTileSourceMask = 0x20000000;

        private const string StateFileName = "state.json";
        private const string ClutFileName = "clut.bin";

        private readonly object sync = new object();
        private readonly IMachine machine;
        private readonly IBusController systemBus;
        private readonly uint[] registers;
        private readonly uint[] committedRegisters;
        private readonly LimitTimer frameTimer;
        private bool frameEventsEnabled;
        private bool committed;

        private const int SharedHeaderBytes = 64;
        private const long SharedMaximumPixels = 1024 * 1024;
        private const uint SharedMagic = 0x46554344; // "DCUF" little-endian
        private readonly object publishSync = new object();
        private MemoryMappedFile sharedFile;
        private MemoryMappedViewAccessor sharedView;
        private ulong publishIntervalTicks;
        private ulong lastPublishTicks;
        private bool hasPublished;
        private Task composeTask;
    }
}
