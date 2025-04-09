all: loados.decb

SOURCES= bonobo.c console.c console.h loados.h main.c network.c prelude.c prelude.h loados.script loados.c
GCCFLAGS = -Os -g -fwhole-program -fomit-frame-pointer

loados.decb: $(SOURCES)
	gcc6809 -std=gnu99 -S -I. $(GCCFLAGS) loados.c
	gcc6809 -std=gnu99 -c -I. $(GCCFLAGS) loados.c
	lwlink --decb  -o'loados.decb'  *.o  --entry=entry --script=loados.script --map=loados.map
	grep ^Section: loados.map

clean:
	rm -f _* ,* *.o *.s *.list *.map loados.decb
