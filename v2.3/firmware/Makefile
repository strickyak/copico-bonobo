all: life-semi.python bonobo-step52.py

PRAGMATA = --pragma=undefextern --pragma=cescapes --pragma=importundefexport

life-semi.s: life-semi.c
	$$HOME/coco-shelf/bin/gcc6809 -S -O2 -std=gnu99 $<
life-semi.o: life-semi.s
	$$HOME/coco-shelf/bin/lwasm --format=obj $(PRAGMATA) -o'$@' $<
entry.o: entry.asm
	$$HOME/coco-shelf/bin/lwasm --format=obj $(PRAGMATA) -o'$@' $<
life-semi.decb: life-semi.o entry.o
	$$HOME/coco-shelf/bin/lwlink -obj --script=entry0200.script --entry=entry --output=$@ $< entry.o --map=life-semi.map
life-semi.python: life-semi.decb decb_to_word_pairs
	./decb_to_word_pairs < $< > $@

bonobo-step52.py: bonobo-step51.py
	cat life-semi.python > $@
	sed -n '/BONOBO/,$$ p' bonobo-step51.py >> $@
