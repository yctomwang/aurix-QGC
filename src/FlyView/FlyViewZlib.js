// Minimal pure-JS zlib inflate (RFC 1950 wrapper + RFC 1951 DEFLATE).
// Ported from tiny-inflate (MIT) for use in QML's JavaScript engine.
// No external dependencies. Operates on Uint8Array input/output.

var TINF_OK = 0;
var TINF_DATA_ERROR = -3;

function Tree() {
    this.table = new Uint16Array(16);
    this.trans = new Uint16Array(288);
}

function Data(source, dest) {
    this.source = source;
    this.sourceIndex = 0;
    this.tag = 0;
    this.bitcount = 0;
    this.dest = dest;
    this.destLen = 0;
    this.ltree = new Tree();
    this.dtree = new Tree();
}

var sltree = new Tree();
var sdtree = new Tree();

var length_bits = new Uint8Array(30);
var length_base = new Uint16Array(30);
var dist_bits = new Uint8Array(30);
var dist_base = new Uint16Array(30);

var clcidx = new Uint8Array([
    16, 17, 18, 0, 8, 7, 9, 6,
    10, 5, 11, 4, 12, 3, 13, 2,
    14, 1, 15
]);

var code_tree = new Tree();
var lengths = new Uint8Array(288 + 32);
var offs = new Uint16Array(16);

function tinf_build_bits_base(bits, base, delta, first) {
    var i, sum;
    for (i = 0; i < delta; ++i) bits[i] = 0;
    for (i = 0; i < 30 - delta; ++i) bits[i + delta] = i / delta | 0;
    for (sum = first, i = 0; i < 30; ++i) {
        base[i] = sum;
        sum += 1 << bits[i];
    }
}

function tinf_build_fixed_trees(lt, dt) {
    var i;
    for (i = 0; i < 7; ++i) lt.table[i] = 0;
    lt.table[7] = 24;
    lt.table[8] = 152;
    lt.table[9] = 112;
    for (i = 0; i < 24; ++i) lt.trans[i] = 256 + i;
    for (i = 0; i < 144; ++i) lt.trans[24 + i] = i;
    for (i = 0; i < 8; ++i) lt.trans[24 + 144 + i] = 280 + i;
    for (i = 0; i < 112; ++i) lt.trans[24 + 144 + 8 + i] = 144 + i;
    for (i = 0; i < 5; ++i) dt.table[i] = 0;
    dt.table[5] = 32;
    for (i = 0; i < 32; ++i) dt.trans[i] = i;
}

function tinf_build_tree(t, lengths, off, num) {
    var i, sum;
    for (i = 0; i < 16; ++i) t.table[i] = 0;
    for (i = 0; i < num; ++i) t.table[lengths[off + i]]++;
    t.table[0] = 0;
    for (sum = 0, i = 0; i < 16; ++i) {
        offs[i] = sum;
        sum += t.table[i];
    }
    for (i = 0; i < num; ++i) {
        if (lengths[off + i]) t.trans[offs[lengths[off + i]]++] = i;
    }
}

function tinf_getbit(d) {
    if (!d.bitcount--) {
        d.tag = d.source[d.sourceIndex++];
        d.bitcount = 7;
    }
    var bit = d.tag & 1;
    d.tag >>>= 1;
    return bit;
}

function tinf_read_bits(d, num, base) {
    if (!num) return base;
    while (d.bitcount < 24) {
        d.tag |= d.source[d.sourceIndex++] << d.bitcount;
        d.bitcount += 8;
    }
    var val = d.tag & (0xffff >>> (16 - num));
    d.tag >>>= num;
    d.bitcount -= num;
    return val + base;
}

function tinf_decode_symbol(d, t) {
    while (d.bitcount < 24) {
        d.tag |= d.source[d.sourceIndex++] << d.bitcount;
        d.bitcount += 8;
    }
    var sum = 0, cur = 0, len = 0;
    var tag = d.tag;
    do {
        cur = 2 * cur + (tag & 1);
        tag >>>= 1;
        ++len;
        sum += t.table[len];
        cur -= t.table[len];
    } while (cur >= 0);
    d.tag = tag;
    d.bitcount -= len;
    return t.trans[sum + cur];
}

function tinf_decode_trees(d, lt, dt) {
    var hlit = tinf_read_bits(d, 5, 257);
    var hdist = tinf_read_bits(d, 5, 1);
    var hclen = tinf_read_bits(d, 4, 4);
    var i, num, length;
    for (i = 0; i < 19; ++i) lengths[i] = 0;
    for (i = 0; i < hclen; ++i) {
        lengths[clcidx[i]] = tinf_read_bits(d, 3, 0);
    }
    tinf_build_tree(code_tree, lengths, 0, 19);
    for (num = 0; num < hlit + hdist;) {
        var sym = tinf_decode_symbol(d, code_tree);
        switch (sym) {
        case 16:
            var prev = lengths[num - 1];
            for (length = tinf_read_bits(d, 2, 3); length; --length)
                lengths[num++] = prev;
            break;
        case 17:
            for (length = tinf_read_bits(d, 3, 3); length; --length)
                lengths[num++] = 0;
            break;
        case 18:
            for (length = tinf_read_bits(d, 7, 11); length; --length)
                lengths[num++] = 0;
            break;
        default:
            lengths[num++] = sym;
            break;
        }
    }
    tinf_build_tree(lt, lengths, 0, hlit);
    tinf_build_tree(dt, lengths, hlit, hdist);
}

function tinf_inflate_block_data(d, lt, dt) {
    while (1) {
        var sym = tinf_decode_symbol(d, lt);
        if (sym === 256) return TINF_OK;
        if (sym < 256) {
            d.dest[d.destLen++] = sym;
        } else {
            sym -= 257;
            var length = tinf_read_bits(d, length_bits[sym], length_base[sym]);
            var dist = tinf_decode_symbol(d, dt);
            var o = d.destLen - tinf_read_bits(d, dist_bits[dist], dist_base[dist]);
            for (var ii = o; ii < o + length; ++ii)
                d.dest[d.destLen++] = d.dest[ii];
        }
    }
}

function tinf_inflate_uncompressed_block(d) {
    while (d.bitcount > 8) {
        d.sourceIndex--;
        d.bitcount -= 8;
    }
    var length = d.source[d.sourceIndex + 1];
    length = 256 * length + d.source[d.sourceIndex];
    var invlength = d.source[d.sourceIndex + 3];
    invlength = 256 * invlength + d.source[d.sourceIndex + 2];
    if (length !== (~invlength & 0x0000ffff)) return TINF_DATA_ERROR;
    d.sourceIndex += 4;
    for (var i = length; i; --i)
        d.dest[d.destLen++] = d.source[d.sourceIndex++];
    d.bitcount = 0;
    return TINF_OK;
}

function rawInflate(source, dest) {
    var d = new Data(source, dest);
    var bfinal, btype, res;
    do {
        bfinal = tinf_getbit(d);
        btype = tinf_read_bits(d, 2, 0);
        switch (btype) {
        case 0:
            res = tinf_inflate_uncompressed_block(d);
            break;
        case 1:
            res = tinf_inflate_block_data(d, sltree, sdtree);
            break;
        case 2:
            tinf_decode_trees(d, d.ltree, d.dtree);
            res = tinf_inflate_block_data(d, d.ltree, d.dtree);
            break;
        default:
            res = TINF_DATA_ERROR;
        }
        if (res !== TINF_OK) return null;
    } while (!bfinal);

    if (d.destLen < d.dest.length) {
        return d.dest.subarray(0, d.destLen);
    }
    return d.dest;
}

// Init static tables
tinf_build_fixed_trees(sltree, sdtree);
tinf_build_bits_base(length_bits, length_base, 4, 3);
tinf_build_bits_base(dist_bits, dist_base, 2, 1);
length_bits[28] = 0;
length_base[28] = 258;

// Zlib inflate: strips the 2-byte zlib header (CMF, FLG) and optional
// FDICT (4 bytes), then delegates to raw DEFLATE inflate.
// The output buffer is sized generously (10x input) then trimmed.
function zlibInflate(compressed) {
    if (!compressed || compressed.length < 6) return null;
    var flg = compressed[1];
    var offset = 2;
    if (flg & 0x20) offset += 4;  // FDICT present

    var deflateData = compressed.subarray(offset, compressed.length - 4); // strip adler32 trailer

    // Point cloud frames are at most ~96 KB uncompressed (6000 pts × 16 B + 4 B header).
    // Use 20× input as generous upper bound; retry with 40× if first attempt fails.
    var maxOut = deflateData.length * 20;
    if (maxOut < 131072) maxOut = 131072;
    var outBuf = new Uint8Array(maxOut);
    var result = rawInflate(deflateData, outBuf);
    if (!result) {
        maxOut = deflateData.length * 40;
        outBuf = new Uint8Array(maxOut);
        result = rawInflate(deflateData, outBuf);
    }
    return result;
}
