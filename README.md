# driver-create-sys-file
automation create sys file in driver
这是一个rtc（rtc-rx8731）驱动，中间有用到在/sys底下创建file结点
实现在sys目录下自动创建file
例如你想修改一个参数值，可以通过echo > 来修改．　cat 来读取
