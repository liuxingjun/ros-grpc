# 帮助信息
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  help        Display this help message"
	@echo "  build       Build the project"
	@echo "  clean       Clean up build artifacts"

# 构建目标
.PHONY: build
build: 
	colcon build --merge-install --packages-up-to ros_grpc --mixin ccache --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 清理构建产物
.PHONY: clean
clean:
	rm -rf install build log
