SHELF := $(shell (cd ../../../ && pwd))

all: decb_to_word_pairs.bin build-c/_loados.decb.h build-c-bonobo-spoonfed/bonobo-spoonfed.uf2 build-c-bonobo-spoonlessraw/bonobo-spoonlessraw.uf2

##############PICO_ENV = PICO_EXAMPLES_PATH=/dev/null PICO_SDK_PATH=../../../../pico-sdk
PICO_ENV = PICO_EXAMPLES_PATH=/dev/null PICO_SDK_PATH=$(SHELF)/pico-sdk SSHHEELLFF=$(SHELF)

decb_to_word_pairs.bin: decb_to_word_pairs.c
	$(CC) $(CFLAGS) -o decb_to_word_pairs.bin decb_to_word_pairs.c

loados/loados.decb: FORCE
	make -C loados

build-c/_loados.decb.h: decb_to_word_pairs.bin loados/loados.decb
	mkdir -p build-c
	./decb_to_word_pairs.bin "c" \
        < loados/loados.decb \
        > build-c/_loados.decb.h

build-c-bonobo-spoonfed/bonobo-spoonfed.uf2: build-c/_loados.decb.h FORCE
	mkdir -p build-c-bonobo-spoonfed
	cd build-c-bonobo-spoonfed && $(PICO_ENV) cmake ../c/bonobo-spoonfed
	make -j4 -C build-c-bonobo-spoonfed
	ls -l build-c-bonobo-spoonfed/bonobo-spoonfed.uf2

build-c-bonobo-spoonlessraw/bonobo-spoonlessraw.uf2: build-c/_loados.decb.h FORCE
	mkdir -p build-c-bonobo-spoonlessraw
	cd build-c-bonobo-spoonlessraw && $(PICO_ENV) cmake ../c/bonobo-spoonlessraw
	make -j4 -C build-c-bonobo-spoonlessraw
	ls -l build-c-bonobo-spoonlessraw/bonobo-spoonlessraw.uf2

clean:
	rm -f *.bin *.list *.map *.o *.s *.decb *.tmp loados/loados.decb
	rm -rf build-*

FORCE:
