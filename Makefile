.PHONY: all module clean lint test frontend

BIN     = bin/teach-frames
TARBALL = bin/module.tar.gz

GO_BUILD_ENV :=
GOFLAGS = -trimpath
LDFLAGS = -s -w

all: $(BIN)

$(BIN): Makefile $(shell find . -type f -name '*.go')
	mkdir -p bin
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GOFLAGS) -ldflags '$(LDFLAGS)' -o $(BIN) ./cmd/module

module: $(BIN) frontend
	tar -czf $(TARBALL) $(BIN) meta.json frontend/dist

frontend:
	cd frontend && npm ci && npm run build

lint:
	go vet ./...
	gofmt -l . | (! grep .)

test:
	go test ./...

clean:
	rm -rf bin
