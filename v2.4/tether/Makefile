all: install

install: cross-compile
	rm -fv ../../../bin/tether
	rm -fv ../../../bin/tether.*.exe
	cp -fv tether.*.exe ../../../bin/

STATICLY =    -ldflags '-extldflags "-static"'

cross-compile:
	GOOS=linux GOARCH=amd64       go build -x $(STATICLY) -o tether.linux-amd64.exe tether.go
	GOOS=linux GOARCH=386         go build -x $(STATICLY) -o tether.linux-386.exe tether.go
	GOOS=linux GOARCH=arm GOARM=7 go build -x $(STATICLY) -o tether.linux-arm-7.exe tether.go
	GOOS=linux GOARCH=arm64       go build -x $(STATICLY) -o tether.linux-arm64.exe tether.go
	GOOS=windows GOARCH=amd64     go build -x $(STATICLY) -o tether.win-amd64.exe tether.go
	GOOS=windows GOARCH=386       go build -x $(STATICLY) -o tether.win-386.exe tether.go
	GOOS=darwin GOARCH=arm64      go build -x $(STATICLY) -o tether.mac-arm64.exe tether.go
	GOOS=darwin GOARCH=amd64      go build -x $(STATICLY) -o tether.mac-amd64.exe tether.go
	rm -f tether-all.zip
	zip tether-all.zip tether.*.exe

clean:
	rm -f tether tether.*.exe tether-all.zip
