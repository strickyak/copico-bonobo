# this is about fixing the coco3 boot problems.

from gime_scrape import nums

# Remember the last value in each registger.
D = dict()
for (a, b) in nums:
    D[a] = b

# Print only those in the GIME range.
for k in sorted(D.keys()):
    if 0xFF90 <= k < 0xFFC0:
        print( "  { 0x%04x, 0x%02x }," % ( k, D[k]) )
pass
