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

# Build the bundled Svelte app. In local dev `npm` is already on PATH. In the
# Viam cloud build it is not — setup.sh installs Node via mise, but the build
# step runs in a fresh shell where mise is not activated, so fall back to mise's
# shims (and mise itself) when npm is missing.
frontend:
	cd frontend && \
	if ! command -v npm >/dev/null 2>&1 && [ -d "$(HOME)/.local/share/mise/shims" ]; then \
		export PATH="$(HOME)/.local/share/mise/shims:$(HOME)/.local/bin:$$PATH"; \
	fi; \
	npm ci && npm run build

lint:
	go vet ./...
	gofmt -l . | (! grep .)

test:
	go test ./...

clean:
	rm -rf bin
