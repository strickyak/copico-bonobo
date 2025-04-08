all: loados.decb

SOURCES= bonobo.c console.c console.h loados.h main.c network.c prelude.c prelude.h loados.script

loados.decb: $(SOURCES)
	gcc6809 -std=gnu99 -c -I. *.c
	lwlink --decb  -o'loados.decb'  *.o  --entry=entry --script=loados.script --map=loados.map
	head loados.map

clean:
	rm -f _* ,* *.o *.s *.list *.map loados.decb
