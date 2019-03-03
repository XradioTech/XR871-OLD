
## xr871sdk-release-1.4.0
* 系统
	* 声波配网方案默认选择V2版本，优化策略提高配网成功率
	* 减少解压缩模块内存占用
	* bootloader支持app.bin.xz解压缩功能，即支持app.bin压缩打包
	* 优化部分libc函数实现
	* OTA支持附加检验的功能
	* 修复exception handler无法执行问题
* 应用示例
	* 增加button的app及example:
    	* project/common/apps/buttons
    	* project/example/button
	* 增加music player的app及example:
    	* project/common/apps/player_app
    	* project/example/player
	* 增加music recorder的app及example:
    	* project/common/apps/recorder_app
    	* project/example/record
* 音乐播控
	* 统一libcedarx.a
	* 增加API用于选择cedarx支持的功能， 参考platform_init实现
	* 支持播放WAV格式
	* 支持https证书认证功能
	* 支持混响
	* 内存和功能优化

* 驱动
	* UART：优化驱动实现，解决DMA发送丢失问题
	* GPIO：优化驱动实现，支持重复初始化，增加相关API
	* DMA：增加指定channel的获取
	* flash：优化驱动code size
	* I2S：修复播放噪音问题
	* PWM：支持输出24MHz，修复若干bug
	* sd card：提高3.0卡的兼容性
	* 修复HAL_UDelay()定时不准问题
	* 使能除零异常

* WiFi
	* 增加接口扫描指定的信道
	* 优化启动加载速度
	* 修复休眠唤醒后的内存泄漏问题

* 网络
	* 修复HTTPC若干Bug，增强兼容性
	* 优化SNTP实现
	* 解决HTTPS OTA升级失败问题
	* 修复nopoll若干Bug
	* 修复mbedtls若干Bug

* 工具
	* mkimage：支持生成OTA附件检验信息
	* 声纹配网apk：支持两种配网策略选择
