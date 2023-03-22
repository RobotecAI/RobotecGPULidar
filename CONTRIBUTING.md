# Contribution guidelines and standards

## C++ coding style

### ClangFormat
[ClangFormat](https://clang.llvm.org/docs/ClangFormat.html) is a widely-used C++ code formatter. RGL uses **ClangFormat 14.0.0** to make the resulting code more consistent and readable.

Changes made to Robotec GPU Lidar should follow the established formatting style defined in the [.clang-format file](https://github.com/RobotecAI/RobotecGPULidar/blob/main/.clang-format).

### Working with clang-format

To install clang-format on Ubuntu 22.04, follow the steps described in the [instructions](https://installati.one/install-clang-format-14-ubuntu-22-04/).

To reformat all C/C++ files in the repository, follow these instructions:
```bash
find src/ test/ include/ tools/ | grep -E '.*\.(c|h|cpp|hpp|cu|cuh)$' | xargs clang-format -i
```

### Git hook
You can configure a pre-commit hook that will prevent you from committing unformatted C/C++ files by:

```bash
git config core.hooksPath .githooks
```

If you use your own git hook, you can alternatively copy ours or overwrite your pre-commit hook:

```bash
cat .githooks/pre-commit >> .git/hooks/pre-commit
```

