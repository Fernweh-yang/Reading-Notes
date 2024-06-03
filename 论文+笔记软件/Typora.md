# Mermaid
使用Mermaid可以画UML，甘特图，流程图等等
[知乎参考](https://zhuanlan.zhihu.com/p/172635547)
[官网参考](https://mermaid.js.org/syntax/flowchart.html)

- 如何给图片加标题
`<center style="color:#C125C0C0">图4</center>`: <center style="color:#C125C0C0">图4</center>

# PicGo+Typora+Github

[参考](https://zhuanlan.zhihu.com/p/168729465)

- 下载PicGo:

  https://github.com/Molunerfinn/PicGo/releases/tag/v2.3.1

  下载正式版本的.appimage版本

- 在typora中先选中文，再在图像中选择上传服务为PicGo(app)

  在路径中选择上面下载的.appimage文件

- 给予.appimage权限

  ```
  sudo chmod u+x PicGo-2.3.1.AppImage
  ```

- 启动picgo后设置图床

  1. 仓库名：

     ```
     Fernweh-yang/ImageHosting
     ```

  2. 分支名

     ```
     main
     ```

  3. token即github生成的密码

  4. 存储路径

     ```
     img
     ```

     