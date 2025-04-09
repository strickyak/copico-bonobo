all: decb_to_word_pairs.elf build-c/_kernel.decb.h build-c/_loados.decb.h
	sh compile-and-wrap-with-py.sh  life-semi.c bonobo-life-semi.py
	./decb_to_word_pairs.elf "c" < life-semi.decb > demo-life-semi-c/life-semi.decb.for-c.h

decb_to_word_pairs.elf: decb_to_word_pairs.c
	$(CC) $(CFLAGS) -o decb_to_word_pairs.elf decb_to_word_pairs.c

loados/loados.decb: FORCE
	make -C loados

build-c/_loados.decb.h: loados/loados.decb
	./decb_to_word_pairs.elf "c" \
        < loados/loados.decb \
        > build-c/_loados.decb.h

build-c/_kernel.decb.h: ../../../nekotos/build-for-16k-bonobo
	./decb_to_word_pairs.elf "c" \
        < ../../../nekotos/build-for-16k-bonobo/_kernel.decb \
        > build-c/_kernel.decb.h

clean:
	rm -f *.elf *.list *.map *.o *.s *.decb *.tmp

FORCE:

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
