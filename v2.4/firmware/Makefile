all: decb_to_word_pairs.elf
	sh compile-and-wrap-with-py.sh  life-semi.c bonobo-life-semi.py

decb_to_word_pairs.elf: decb_to_word_pairs.c
	$(CC) $(CFLAGS) -o decb_to_word_pairs.elf decb_to_word_pairs.c

clean:
	rm -f *.elf *.list *.map *.o *.s *.decb *.tmp

##  all: life-semi.decb life-semi.python bonobo-step52.py
##  
##  PRAGMATA = --pragma=undefextern --pragma=cescapes --pragma=importundefexport
##  
##  life-semi.s: life-semi.c
##  	$$HOME/coco-shelf/bin/gcc6809 -S -O2 -std=gnu99 $<
##  life-semi.o: life-semi.s
##  	$$HOME/coco-shelf/bin/lwasm --format=obj $(PRAGMATA) -o'$@' $<
##  entry.o: entry.asm
##  	$$HOME/coco-shelf/bin/lwasm --format=obj $(PRAGMATA) -o'$@' $<
##  life-semi.decb: life-semi.o entry.o
##  	$$HOME/coco-shelf/bin/lwlink -obj --script=entry0200.script --entry=entry --output=$@ $< entry.o --map=life-semi.map
##  life-semi.python: life-semi.decb decb_to_word_pairs
##  	./decb_to_word_pairs < $< > $@
##  
##  bonobo-step52.py: bonobo-step51.py life-semi.python
##  	cat life-semi.python > $@
##  	sed -n '/BONOBO/,$$ p' bonobo-step51.py >> $@
