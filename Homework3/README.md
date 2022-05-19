已完成：
1. 代码可以正常编译、执行
2. 在rasterize_triangle函数中，判断像素是否在三角形内，实现深度、颜色、法线、纹理坐标等插值，把插值信息传入fragment_shader获取像素颜色，判断替换插值深度与z-buffer，设置像素颜色。
3. 使用Bling-Phong模型，在for循环中依次计算环境光、漫反射光及高光
4. kd视为纹理颜色，使用Bling-Phong模型
5. 分别实现注释中的伪代码，实现bump mapping与displacement mapping

未完成提高部分

图片均在/images文件夹中