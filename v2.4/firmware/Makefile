all: decb_to_word_pairs.bin build-c/_loados.decb.h build-c/bonobo.uf2

PICO_ENV = PICO_EXAMPLES_PATH=/dev/null PICO_SDK_PATH=../../../../pico-sdk

decb_to_word_pairs.bin: decb_to_word_pairs.c
	$(CC) $(CFLAGS) -o decb_to_word_pairs.bin decb_to_word_pairs.c

loados/loados.decb: FORCE
	make -C loados

build-c/_loados.decb.h: decb_to_word_pairs.bin loados/loados.decb
	mkdir -p build-c
	./decb_to_word_pairs.bin "c" \
        < loados/loados.decb \
        > build-c/_loados.decb.h

build-c/bonobo.uf2: build-c/_loados.decb.h FORCE
	mkdir -p build-c
	cd build-c && $(PICO_ENV) cmake ../c/
	make -j4 -C build-c
	ls -l build-c/bonobo.uf2

clean:
	rm -f *.bin *.list *.map *.o *.s *.decb *.tmp loados/loados.decb
	rm -rf build-*

FORCE:
