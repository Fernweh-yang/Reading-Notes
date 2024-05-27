[yaml官网](https://yaml.org/)

# 基本语法：

- 大小写铭感
- 使用缩进表示层级关系
- 缩进不允许使用tab,只允许空格，空格数不重要，只要求同级元素左对齐
- \#表示注释

# 数据类型

- 对象：键值对的集合。又可称为映射mapping,哈希hashes和字典dictionary
- 数组：一组按次序排列的值，又称为序列sequence或列表list
- 纯量scalars：单个不可再分的值

## YAML对象

- 三种表现形式

```yaml
# 1.用冒号结构表示，冒号后加一空格
key: value
# 2.映射的形式
key:{key1: value1,key2: value2,..}
# 3.
key:
	child-key: value
	child-key2: value2
```

- 复杂对象格式  ?:

```
?
	- complexkey1
	- complexkey2
:
	- complexvalue1
	- complexvalue2
```

对象的键是一个数组[complexkey1,complexkey2]

对象的值也是一个数组[complexvalue1,complexvalue2]

## YAML数组

- 以-开头的行表示构成一个数组

  ```yaml
  - a
  - b
  - c
  ```

- 多维数组行内表示:

  ```yaml
  key: [value1, value2,...]
  ```

- 数据结构的子成员是一个数组，则可以在该项下面缩进一个空格。

  ```yaml
  - 
   - A
   - B
   - C
  ```

  ```yaml
  waypoints1:
     - x:0.0
     - y:0.0
     - z:0.0
     
  ```

  

- 例子

  ```yaml
  companies:
      -
          id: 1
          name: company1
          price: 200W
      -
          id: 2
          name: company2
          price: 500W
  ```

  - companies是对象的键，他的值是一个数组
  - 每个数组元素又由3个对象构成。

## 纯量

- 例子：

  数组里的值就是一个个纯量，不可再分

```yaml
boolean: 
    - TRUE  #true,True都可以
    - FALSE  #false，False都可以
float:
    - 3.14
    - 6.8523015e+5  #可以使用科学计数法
int:
    - 123
    - 0b1010_0111_0100_1010_1110    #二进制表示
null:
    nodeName: 'node'
    parent: ~  #使用~表示null
string:
    - 哈哈
    - 'Hello world'  #可以使用双引号或者单引号包裹特殊字符
    - newline
      newline2    #字符串可以拆成多行，每一行会被转化成一个空格
date:
    - 2018-02-17    #日期必须使用ISO 8601格式，即yyyy-MM-dd
datetime: 
    -  2018-02-17T15:02:31+08:00    #时间使用ISO 8601格式，时间和日期之间使用T连接，最后使用+代表时区
```

# 引用

- &锚点和*别名

```yaml
defaults: &defaults
  adapter:  postgres
  host:     localhost

development:
  database: myapp_development
  <<: *defaults
  
#相当于
defaults:
  adapter:  postgres
  host:     localhost

development:
  database: myapp_development
  adapter:  postgres
  host:     localhost
```

**&** 用来建立锚点（defaults），**<<** 表示合并到当前数据，***** 用来引用锚点。

```
- &showell Steve 
- Clark 
- Brian 
- Oren 
- *showell 
```

