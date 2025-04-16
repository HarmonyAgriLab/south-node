# 南向设备节点代码
## 1 仓库目录

hi3863-nb-mesh存放的是已经验证过的自组网+NB的代码，但仍有很多细节需要处理，未来优化时可以参考这份代码

~~~
south-node/
├── hi3863-nb-nomesh/    # 没有mesh，NB直连版本代码
├── hi3863-nb-mesh/      # mesh版本代码
~~~

## 2 环境配置

本项目的开发在WSL上，关于WSL子系统的环境配置请看文档[WSL子系统开发环境搭建](environment/WSL子系统开发环境搭建.md)，在参考这个文档成功配置好WSL的环境后，请参考这个文档配置SDK的编译开发环境[WSL子系统编译及烧录](environment/WSL子系统编译及烧录.md)。

## 3 hi3863-nb-nomesh版本部署

详情请参考[hi3863-nb-nomesh README 中的部署说明](hi3863-nb-nomesh/README.md)

## 4 hi3863-nb-mesh版本部署

详情请参考[hi3863-nb-mesh README 中的部署说明](hi3863-nb-mesh/README.md)

