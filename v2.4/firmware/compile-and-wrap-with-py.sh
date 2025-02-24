#!/bin/sh

set -eux

PRAGMATA='--pragma=undefextern --pragma=cescapes --pragma=importundefexport'

alias gcc="$HOME/coco-shelf/bin/gcc6809"
alias lwasm="$HOME/coco-shelf/bin/lwasm"
alias lwlink="$HOME/coco-shelf/bin/lwlink"

case $# in
    2) : good ;;
    *) echo "Usage:   $0 c-file.c python-wrapper.py" >&2
       exit 13 ;;
esac

C=$(basename $1 .c)
P=$(basename $2 .py)

gcc -S -O2 -std=gnu99 $C.c
lwasm --format=obj ${PRAGMATA} -o"$C.o" --list="$C.o.list" $C.s
lwlink -obj --script=entry0200.script --entry=ENTRY \
    --output="$C.decb" --map="$C.map" "$C.o"

make decb_to_word_pairs.elf
./decb_to_word_pairs.elf <"$C.decb"  > "$C.tmp"

(
    cat "$C.tmp"
	sed -n '/BONOBO/,$ p' $P.py
) > /tmp/$P.tmp
mv -fv /tmp/$P.tmp $P.py
