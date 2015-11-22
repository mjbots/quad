#!/usr/bin/python
"""H264 bytestream decoder

(H264 comes in bytestream and other (raw NAL? AU? rbsp?) formats.
This only deals with byte streams, which have embedded resync markers.

the full decoder to text (in C) can be downloaded from:
http://sourceforge.net/projects/h264bitstream/
it shows some interesting things for C920 streams:
 - there are 16 slices per frame
 - there are only P and I frames with default settings

Note: H264 spec may be downloaded from
https://www.itu.int/rec/T-REC-H.264-201402-I/en

Other refs:
http://stackoverflow.com/questions/5528712/h-264-bytestream-parsing
https://cardinalpeak.com/blog/the-h-264-sequence-parameter-set/

"""
import sys
import binascii

def read_nals(fh):
    buff = ''
    nonfirst = 0
    while True:
        block = fh.read(65536)
        buff += block
        while True:
            next_start = buff.find('\x00\x00\x01', nonfirst)
            if next_start == -1:
                break
            if next_start > 0 and buff[next_start - 1] == '\x00':
                next_start -= 1
            if not nonfirst:
                if next_start != 0:
                    print >>sys.stderr, 'skipping junk in front (%d): %r' % (
                        next_start, buff[:next_start][:64])
                    buff = buff[next_start:]
                nonfirst = 2
                continue

            if buff.startswith('\x00\x00\x01'):
                hdrlen = 3
            elif buff.startswith('\x00\x00\x00\x01'):
                hdrlen = 4
            else:
                assert False
            #print next_start, hdrlen, repr(buff[:64])
            yield buff[:hdrlen], buff[hdrlen:next_start]
            buff = buff[next_start:]
        if block == '':
            break

# from Table 7.1, pg 85
UNIT_TYPES = [
    'Unspec',     # 0
    'Slice',
    'SliceA',
    'SlideB',
    'SliceC',     # 4
    'SliceIDR',   # I-frame slice
    'SEI',    # the only thing which contains timing info?
    'SPS',    # parameter sets
    'PPS',    # 8
    'AUI',
    'SeqEnd',
    'StreamEnd',
    'Filler',  # 12
    'SPS-E',
    'PrefNal',
    'S-SPS',
    'DPS',     # 16
    'R_17', 'R_18',
    'Slice-aux',
    'Slice-ext',  # 20
    'Slice-3D',
    'R_22', 'R-23',
] + [ 'unspec%d' for d in xrange(24, 32) ]
assert len(UNIT_TYPES) == 32

def main():
    slice_group = None
    slice_lens = []
    slice_types = set()
    def flush_slices():
        if not slice_lens:
            return
        total_len = sum(slice_lens)
        print '%16s: %d NALs, len %d..%d bytes (total %d, avg %.1f)' % (
            '+'.join(sorted(slice_types)), len(slice_lens),
            min(slice_lens), max(slice_lens), total_len,
            total_len * 1.0 / len(slice_lens))
        slice_lens[:] = []
        slice_types.clear()

    for nal_hdr, nal_bin in read_nals(open(sys.argv[1], 'rb')):
        # decode NAL
        b1 = ord(nal_bin[0])
        assert (b1 & 0x80) == 0
        nal_len = len(nal_hdr) + len(nal_bin)
        ref_idc = (b1 >> 5) & 3
        unit_type = b1 & 31
        type_str = '%s[%d]' % (UNIT_TYPES[unit_type], ref_idc)
        # to decode more, you need to start doing bit ops and Exp-Golomb
        # numbers. I am too lazy for that.

        # The good thing to decode is type 1, 'Slice':
        #  it is slice_layer_without_partitioning_rbsp(), 7.3.2.8
        #  it starts with slice_header(), 7.3.3/7.4.3
        #  the header contains:
        #     frame_num -- groups slices for one picture together
        #     slice_type (P/B/I/SP/SI)
        #

        # print NAL
        s_group = (1 if unit_type in [1, 2, 3, 4] else
                   2 if unit_type == 5 else None)
        if s_group != slice_group:
            flush_slices()
        if s_group:
            slice_lens.append(nal_len)
            slice_types.add(type_str)
            slice_group = s_group
            continue
        flush_slices()
        print '%16s: len %d bytes' % (type_str, nal_len)
    flush_slices()

if __name__ == '__main__':
    main()
