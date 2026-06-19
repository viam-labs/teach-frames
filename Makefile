BIN = bin/teach-frames

build:
	go build -o $(BIN) ./cmd/module

test:
	go test ./...

lint:
	gofmt -l .
	go vet ./...

module.tar.gz: build
	tar czf module.tar.gz $(BIN) meta.json
