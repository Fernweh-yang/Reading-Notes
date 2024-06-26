# C 杂记
- 字符串

  C语言中，字符串实际上是使用 **null** 字符 **\0** 终止的一维字符数组。

  ```C
  char site[7] = {'R', 'U', 'N', 'O', 'O', 'B', '\0'};
  char site[] = "RUNOOB";
  ```

- 定义数组大小

  ```c
  /*错误：error array index is not constant
  因为，在C语言中const不代表constan，只是说明这个是read-only
  */
  int const int x = 5;
  int arr[x] = {1, 2, 3, 4, 5};
  
  //正确1：
  #define x 5
  int arr[x] = {1, 2, 3, 4, 5};
  //正确2:
  enum { LEN = 5 };
  int arr[LEN] = {1, 2, 3, 4, 5};
  ```

- 把int变量赋值给char数组

  ```c
  char p[10];
  int i =0;
  p[0]=(char)('0'+i);
  ```

- C语言中无bool类型定义，需要自己写

  ```c
  typedef enum
  {
      true=1, false=0
  }bool;
  ```

- 结构Struct

  ```c
  //用typedef可以当一个数据类型来用
  typedef struct{
      int day;
      char *month;
      int year;
  }date;
  date today ={4,"may",2021};
  
  today.day = 4
      ..
  ```

- 位域Bit-fields(1)

  存储某些信息不需要整个字节时使用

  ```c
  # define BIT0 01
  # define BIT1 02
  # define BIT2 04
  
  //或者
  enum {BIT0 = 01, BIT1 = 02, BIT2 = 04};
  ```

## 文件

https://blog.csdn.net/afei__/article/details/81835684

- 打开关闭一个文件

```c
//declare a file
File *fp;

//open or close a file
/*
r:reading the file from the beginning; returns an errorif the file does not exist
w:writing to the file from the beginning; creates the file if it does not exist
a:appending to the end of the file; creates the file if it does not exist.
*/
FILE *fp = fopen (" hello . txt", "w");
fclose(fp);
```

- 读取写入

  C语言fprintf()和fscanf()函数

  - `fprintf()`函数用于将一组字符写入文件。它将格式化的输出发送到流。

  ```c
  int fprintf(FILE *stream, const char *format [, argument, ...])
  ```

  - `fscanf()`函数用于从文件中读取一组字符。它从文件读取一个单词，并在文件结尾返回`EOF`。
  - 打开文件要用r模式，不然无法正确读取。
  
  ```c
  int fscanf(FILE *stream, const char *format [, argument, ...])
  ```
  
  - 例子
  
    ```c
    #include <stdio.h>  
    main() {
        FILE *fp;
        fp = fopen("file.txt", "w");//opening file  
        fprintf(fp, "Hello file by fprintf...\n");//writing data into file  
        fclose(fp);//closing file  
        printf("Write to file : file.txt finished.");
        
        char buff[255];//creating char array to store data of file  
      	fp = fopen("file.txt", "r");  
      	while(fscanf(fp, "%s", buff)!=EOF){  
       		printf("%s ", buff );  
        }  
        fclose(fp);  
    }
    ```
  
    

# C++实践课

## Tag1

1. **Was ist der Unterschied zwischen der dynamischen und statischen Speicherreservierung?**

   | dynamischen                            | statischen                                |
   | -------------------------------------- | ----------------------------------------- |
   | zur Laufzeit Speicher reserviert       | Vor Laufzeit(Compile) Speicher reserviert |
   | Die Speicher wird im Laufzeit befreit. | Die Speicher wird nach Laufzeit befreit.  |

   C中用malloc()和free()

   c++中用new和 delete，new 相比malloc更强因为他会同时构造对象

   ```c++
   // Dynamische Speicherreservierung für eine Variable vom Typ double
   double* pdWidth = new double;
   // Dynamische Speicherreservierung für ein Array der Dimension 80 vom Typ char 
   char* pchName = new char[80];
   // Freigeben einer dynamisch angelegten Variable
   delete pdWidth; 
   // Freigeben eines dynamisch angelegten Arrays 
   delete[] pchName;
   ```

   

2. **Wann benutzt man dynamische Speicherreservierung?**

   Wenn es vor der Laufzeit eines Programms nicht bekannt ist, wie viel Speicher für die Programmausführung notwendig ist. 

3. **Welche Gefahren危险 gehen von Pointern aus?**

   Pointer können auch noch existieren, wenn die referenzierte Variable gelöscht ist. Greift man dann auf den Pointer zu, dereferenziert man undefinierten Speicherbereich, was unweigerlich zu einem Fehler führt.

   ```c++
   // 指针的用法1：
   // Zugreifen auf den ersten Buchstaben des Labels 
   char chLetter = *pchLabel; 
   chLetter = pchLabel[0]; 
   chLetter = pchLabel[]; 
   // Zugreifen auf den fünften Buchstaben 
   chLetter = *(pchLabel+4); 
   chLetter = pchLabel[4];
   
   // 指针的用法2：
   // Signatur der Funktion, die den Pointer auf ein Array mit Nutzdaten 
   // erhält, die den Mittelwert daraus berechnet 
   void calculateMean(float* pfMean, int* piData, int iSize); 
   。。。
   int main() {
       float fMean = 0.0; 
       int piData[] = {1, 2, 3, 4, 58, 5, 6, 1, 98, 3}; 
       // Aufruf der Funktion 
       calculateMean(&fMean, piData, 10); 
       return 0;
   }
   ```

   

4. **Welche Vorteile haben Referenzen引用 gegenüber Pointern?**

   - Der Vorteil gegenüber Pointern besteht im Sicherheitsgewinn.

   - Ein weiterer Vorteil gegenüber Pointern liegt in der vereinfachten Parameterübergabe beim Aufruf von Funktionen

     ```c++
     // 引用的用法1：
     // Initialisierung einer Variablen
     int iStudentID = 27002; 
     // Initialisierung auf die Variable
     int& riStudentID = iStudentID;
     
     // 引用的用法2：
     // Definition einer Funktion die eine Referenz und eine Variable als  
     // Uebergabeparameter erwartet 
     void changeStudentID(int& riOldStudentID, int iNewStudentID){ 
         riOldStudentID = iNewStudentID;
     }
     // Die Funktion wird wie folgt in der Main aufgerufen
     int main(){
         int iStudentIDDetlefCruse = 99933;
         changeStudentID(iStudentIDDetlefCruse, 999773);
         return 0;
     }
     ```


## Tag2

p40

1. **Was ist ein Objekt?**

   Ein Objekt ist eine Datenstruktur mit Bezeichner, die sowohl Variablen als auch Funktionen enthält.

   In diesem Kontext nennt man Variablen变量 Attribute属性 und Funktionen函数 Methoden方法.

2. **Woraus besteht es?**

   - Bezeichner： ist der eindeutige Name des Objektes.
   - Attribute ：enthalten Daten, die sich auf das zugehörige Objekt beziehen.
   - Methoden： eines Objektes manipulieren die Attribute dieses Objekts.

3. **Wie hängen Klasse und Objekt zusammen?**

   Jedes Objekt gehört genau einer Klasse an

4. **Wie viele Objekte lassen sich aus einer Klasse erstellen?**

   Mithilfe einer Klasse lassen sich beliebig viele Objekte erstellen。

   Diese Objekte haben alle den gleichen Aufbau.

5. **Können Objekte andere Objekte enthalten?**

   Nein,keine zwei Objekte sind identisch , d. h. jedes Objekt kann eindeutig identifiziert werden.

6. **Was versteht man unter dem Kapselungsprinzip?**

   Kapselung bedeutet, dass die Attribute eines Objektes nur von ihm selbst und keinesfalls von außen geändert werden dürfen.

7. **Wie wird es realisiert?**

   Zugriffsspezifizierer sind die Schlüsselwörter public, private und protected. 

   Sie erlauben es für jede Methode und jedes Attribut anzugeben, wie darauf zugegriffen werden darf.

8. **Was bewirken die Zugriffsspezifizierer private und public?**

   - private:
     - Das Attribut/die Methode kann nur innerhalb der Methoden der eigenen Klassen aufgerufen werden
     - Eine Verwendung ist jedoch innerhalb der eigenen Methoden möglich.
   - public:
     - Das Attribut oder die Methode kann in jeder anderen Methode, auch außerhalb des Objekts aufgerufen werden.
     - Dieser Aufruf geschieht durch:<Objektname>.<Attributbezeichnung>.

9. **Warum greift man nicht direkt auf Attribute zu, sondern verwendet get()- und set()-Methoden?**

   - Der wesentliche Unterschied der get()-Methode zur direkten Ausgabe kann beispielsweise darin liegen, dass die Ausgabe输出 formatiert格式化 wird.
   - Der wesentliche Unterschied der set()-Methoden zur direkten Zuweisung liegt darin, 
     - dass die Werte nicht ohne Überprüfung übernommen werden, sondern von ihnen innerhalb der Methoden überprüft werden können.
     - So kann beispielsweise die Zuweisung einer negativen Zahl zu einem Attribut, welches das Alter angibt, verhindert werden.

10. **Welche Aufgabe hat der Konstruktor构造函数?**

    - Um Objekte mit Hilfe dieses Bauplans(Klasse) zu erstellen
    - Sie haben den Zweck Speicher für das neue Objekt zu reservieren und die Attribute gegebenenfalls zu initialisieren.

11. **Welche Aufgabe hat der Destruktor析构函数?**

    Er zerstört sie Objekte und gibt den von den Objekten belegten Speicher wieder frei.

12. **Weshalb verwendet man überladene重载 Konstruktoren?**

    Manchmal ist es notwendig Objekte mit unterschiedlichen Attributwerten zu initialisieren.

13. **Wann muss man Initialisierungslisten初始化列表 verwenden?**

    Um die Attribute gleich bei der Erzeugung des Objekts zu initialisieren, ergänzt man die Konstruktor-Definition um eine Initialisierungsliste

14. **Was versteht man unter einem Zugriff innerhalb der Klassendefinition?**

    Eine Klasse zugreift innerhalb einer Methodendefinition auf ihre eigenen Attribute und Methoden.

15. **Was versteht man unter einem Zugriff von außen?**

    Aufruf eines Attributes oder einer Methode außerhalb der Klassendefinition, d.h. in den Definitionen anderer Klassen oder Funktionen.

16. **Worin unterscheiden sich diese beiden Zugriffsarten?**

    - Zugriff innerhalb benötigen Sie keinen speziellen Operator
    - Zugriff von außen wird der Punkt- bzw. Pfeil-Operator benötigt
    - Zugriff innerhalb: Kann die Klasse immer auf ihre eigenen Attribute und Methoden zugreifen.
    - Zugriff von außen: Kann nur public Attribute und Methoden zugreifen.

17. **Wann benötigt man den Punkt- und wann den Pfeiloperator?**

    - Hat man einen Pointer auf ein Objekt und will man über diesen auf die Attribute und Methoden eines Objektes zugreifen, benötigt man den Pfeiloperator ->
      - Der Aufruf sieht folgendermaßen aus: pBirthdayPhotos->m_iNumberOfPhotos
    - Spricht man das Objekt direkt oder über eine Referenz an, so benötigt man den Punktoperator.
      - <Objektname>.<Attribut/Methode>

18.  **Static**

    - Statische Inhalte einer Klasse sind in allen Instanzen对象 der Klasse gleich
      - D.h. hat eine Klasse eine statische Methode oder Variable, so wird diese von allen Instanzen der Klasse „geteilt“.所有该类的对象共享这个静态方法或属性
    - statische Inhalte können auch verwendet werden, wenn keine Instanz der Klasse existiert.
    - statische Variablen : static int siAnz (在class.h文件中定义)
      - 必须在class.cpp文件中初始化： int Animal::siAnz = 0;
    - Nicht-statische Inhalte können von statischen Methoden nicht verwendet werden 
      - da statische Methoden auch ohne eine Instanz der Klasse verwendet werden können
    - Eine häufige Verwendung von statischen Variablen und Methoden ist die Identifikation鉴别 von einzelnen Instanzen对象 sowie das Zählen aller vorhandenen Instanzen einer Klasse.

19. **Const**

    - Kennzeichnet标记的 Funktionen, die Member-Variablen nicht verändert. Bei Aufruf kann auf die Variablen nur lesend zugegriffen werden.
    -  const数据成员 只在某个对象生存期内是常量，而对于整个类而言却是可变的。因为类可以创建多个对象，不同的对象其const数据成员的值可以不同
    - 在C++中，const成员变量也不能在类定义处初始化，只能通过构造函数初始化列表进行，并且必须有构造函数
    - 要想建立在整个类中都恒定的常量，应该用类中的枚举常量来实现，或者static cosnt

20. **Was versteht man unter der Überladung重载 von Methoden und Operatoren?**

    - Diese Methoden mit gleichen Namen, aber unterschiedlicher Übergabeparameter, nennt man überladene Methoden.
    - Überladung von Operatoren ermöglicht es dem Programmierer, die entsprechenden Operationen auch für selbst definierte Datentypen zur Verfügung zu stellen. Das Überladen von Operatoren funktioniert prinzipiell wie bei Methoden.

21. **Wann wendet man die Überladung an?**

    Möchte man eine Funktionalität mit unterschiedlicher Art und Anzahl von Parametern umsetzen

22. **Wie überlädt man Methoden bzw. Operatoren?**

    - Methoden:

      - float CalculateCircleArea(float fRadius);
      - double CalculateCircleArea(double dRadius);

    - Operatoren

      ```c++
      // 这里complex是个类
      complex operator+(complex a, complex b) { 
      	return complex(a.getReTeil() + b.getReTeil(), a.getImTeil() + b.getImTeil()); 
      }
      int main(){
          complex a = complex(2.1,2.0); 
          complex b = complex(4.0,3.3); 
          complex c = a+b;
          return 0;
      }
      ```

23. **Was versteht man unter Vererbung继承?** 

    Die Vererbung ist eine Beziehung zwischen zwei Klassen. Die eine Klasse nennt man Elternklasse, die andere Kindklasse.

24. **Was ist eine Elternklasse und Kindklasse?** 

    - Die Kindklasse ist von der Elternklasse abgeleitet. Das heißt die Kindklasse enthält (erbt) alle Attribute und Methoden der Elternklasse.
    - Zusätzlich können innerhalb der Kindklasse noch weitere Attribute und Methoden implementiert werden

25. **Worin liegt der Nutzen der Vererbung?**

    Durch das Prinzip der Vererbung wird ermöglicht, dass Klassen mit ähnlichen Methoden und Attributen voneinander abgeleitet werden können, sie müssen somit nur einmal implementiert werden.

26. **Worin liegt der Unterschied zwischen Überschreibung und Überladung?** 

    - Überschreiben bezeichnet die Redefinition einer vererbten Methode in der Kindklasse
      - Die Signaturen标志 der Methoden in Eltern und Kindklassen unterscheiden sich nicht, jedoch die Definitionen.
    - Überladen bezeichnet die Definition mehrerer Methoden mit demselben Namen jedoch unterschiedlichen Parametern.
      -  Die Methoden haben also verschiedene Signaturen.

27. **Worin liegt der Nutzen der Überschreibung?**

    - Ruft man nur diese Methode durch ein Objekt der Kindklasse auf, so wird die überschriebene Methode verwendet. 
    - Ein Aufruf der Methode durch ein Objekt der Elternklasse ruft die ursprüngliche Methode auf.

28. **Worin liegt der Unterschied zwischen Schnittstellen接口 und abstrakten Klassen抽象类?** 

    - 带有纯虚函数的类称为抽象类，不能被实例化。纯虚函数是在基类中声明的虚函数，它在基类中没有定义，但要求任何派生类都要定义自己的实现方法。在基类中实现纯虚函数的方法是在函数原型后加“=0”
    - 接口是一个概念。它在C++中用抽象类来实现。一个类一次可以实现若干个接口，但是只能扩展一个父类 
    - 类是对对象的抽象，可以把抽象类理解为把类当作对象，抽象成的类叫做抽象类.而接口只是一个行为的规范或规定。抽象类更多的是定义在一系列紧密相关的类间，而接口大多数是关系疏松但都实现某一功能的类中.

    - ！！！！
      - 定义一个函数为虚函数，不代表函数为不被实现的函数。

      - 定义他为虚函数是为了允许用基类的指针来调用子类的这个函数。

      - 定义一个函数为纯虚函数，才代表函数没有被实现。

      - 定义纯虚函数是为了实现一个接口，起到一个规范的作用，规范继承这个类的程序员必须实现这个函数。

    - Eine Schnittstelle in C++ ist eine Klasse, von der sich keine Objekte bilden lassen, weil keine ihrer Methoden definiert ist. Die Methoden sind nur in der Headerdatei deklariert. Aus diesem Grund benötigt die Schnittstelle keinen Konstruktor und Destruktor.接口不能实例化；
    - Rein abstrakte Klassen sind Klassen, die nur rein virtuelle Methoden beinhalten und somit immer Elternklassen sind. Ihre Kinder müssen alle (rein virtuellen) Methoden überschreiben, damit man von ihnen Objekte bilden kann.
    - Der große Vorteil der Schnittstellen liegt darin, dass ein Programm über eine solche Schnittstelle mit vielen Modulen auf definierte Weise kommunizieren kann.
    - Die abstrakte Klasse enthält, genau wie die rein abstrakte Klasse, rein virtuelle Methoden. *Der Unterschied zur rein abstrakten Klasse liegt darin, dass sie auch implementierte Methoden普通的方法 enthält*

    ```c++
    // 基类
    class Shape 
    {
    public:
       // 提供接口框架的纯虚函数
       virtual int getArea() = 0;
       void setWidth(int w)
       {
          width = w;
       }
       void setHeight(int h)
       {
          height = h;
       }
    protected:
       int width;
       int height;
    };
     
    // 派生类
    class Rectangle: public Shape
    {
        public:
       int getArea()
       { 
          return (width * height); 
       }
    };
    ```

    

29. **Was versteht man unter einer rein virtuellen Methode纯虚函数?** 

    - Methoden, die nicht definiert sind, nennt man rein virtuelle Methoden

    - 纯虚函数是一个在基类中声明的虚函数，它在该基类中没有定义具体的操作内容，要求各派生类根据实际需要定义自己的版本，纯虚函数的声明格式为：**virtual 函数类型 函数名(参数表) = 0;**

      ```c++
      // Rein virtuelle Methode deklarieren
      // 虚函数声明只能出现在类定义中的函数原型声明中，而不能在成员函数实现的时候。
      virtual bool checkForUpdates() = 0;
      ```

30. **Was versteht man unter einer virtuellen Methode虚函数?** 

    - 虚函数是实现运行时多态性基础

    - Virtuelle Methoden benötigt man nur dann, wenn man mit Pointern vom Typ der Elternklasse auf die Kindklasse arbeitet
    - Eine Besonderheit der Vererbung ist die Möglichkeit, Pointer vom Typ der Elternklasse auf Kindklassen zu erstellen. `ParentClass* pChild = new ChildClass()`

31. **Was versteht man unter Polymorphie多态?**

    - Polymorphie in C++ bedeutet, dass ein Funktionsaufruf unterschiedliche Funktionen ausführen kann, je nachdem welche Objekttyp die Funktion aufruft.

    - 同一操作作用于不同的类的实例，将产生不同的执行结果，即不同类的对象收到相同的消息时，得到不同的结果。

## Tag3

1. **Was versteht man unter Rekursion递归?*** **Worin liegt der Unterschied zu Iteration迭代?**

   - Die Funktion ruft sich bei einer Rekursion immer wieder selbst auf, bis ein bestimmtes Abbruchkriterium erreicht wird.
   - Da die aufrufende Funktion warten muss, bis die aufgerufene Funktion das Ergebnis zurückliefert, wächst der call stack stetig an.
   - Erst wenn die aufgerufenen Funktionen ihren Wert zurückliefern, werden die Funktionen und ihre Daten vom Stack entfernt.

# C++11常用特性

需要详细介绍的见c++杂记

## 1. 自动类型推导`auto`

```c++
auto x = 42;        // int
auto y = 3.14;      // double
auto z = "Hello";   // const char*
```

## 2. 范围for循环

```c++
std::vector<int> v = {1, 2, 3, 4, 5};
for (auto& elem : v) {
    std::cout << elem << " ";
}
```

## 3. Lambda表达式(匿名函数)

```c++
auto add = [](int a, int b) { return a + b; };
std::cout << add(2, 3); // 输出 5
```

## 4. 智能指针

```c++
std::shared_ptr<int> p1 = std::make_shared<int>(10);
std::unique_ptr<int> p2 = std::make_unique<int>(20);
```

## 5. nullptr关键字

```c++
// 引入新的空指针常量，替代传统的 NULL。
// nullptr是一个实际的空指针类型，而NULL只是一个宏，所以更加的安全和语义明确
int* p = nullptr;
```

## 6. 静态断言`static_assert`

```c++
// 在编译时就进行断言检查,以确保某些条件在编译时得到满足。
// 区别于assert,只在运行时对条件进行检查。
static_assert(sizeof(int) == 4, "Integers must be 4 bytes");
```

## 7. 右值引用`&&` 和 移动语义`std::move`：

```c++
// 提高了对象的转移效率，减少不必要的拷贝。
std::vector<int> v1 = {1, 2, 3};
std::vector<int> v2 = std::move(v1); // v1 现在为空
```

## 8. `std::thread`多线程支持

```c++
std::thread t([]{ std::cout << "Hello from thread"; });
t.join();
```

## 9. `constexpr`常量表达式

```c++
// 在编译时就可以计算，而不是运行时
// 使得在编译期间就能够得到常量的值，提高程序的效率
constexpr int square(int x) {
    return x * x;
}
constexpr int result = square(5);
```

## 10. `std::chrono`时间库

```c++
auto start = std::chrono::high_resolution_clock::now();
// 一些操作
auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
std::cout << "Duration: " << duration << "ms\n";
```

## 11. 统一的初始化语法

```c++
// 使用花括号初始化任何对象，包括内置类型和用户自定义类型。
int x{10};
std::vector<int> v{1, 2, 3, 4, 5};
```

## 12. `enum class`枚举

```c++
// 强类型枚举，避免命名冲突和隐式转换问题。
enum class Color { Red, Green, Blue };	// 默认从0开始递增
Color c = Color::Red;	

int x = Color::Red; // 编译错误：不能将枚举值直接赋给整数
int x = static_cast<int>(Color::Red); // 显式转换为整数
```

## 13. 模板别名`using`

```c++
// 使用 using 关键字还可以为模板创建别名，比 typedef 更灵活
template<typename T>
using Vec = std::vector<T>;
Vec<int> v; // std::vector<int>
```

## 14. 继承构造函数

```c++
class Base {
public:
    Base(int x) {}
};
// 允许在派生类中继承基类的构造函数。
class Derived : public Base {
using Base::Base; // 继承 Base 的构造函数
};

```

# C++17常用特性

## 1. 多线程并行化`std::for_each`

### 基本语法

```c++
for_each( policy, InputIt first, InputIt last, UnaryFunction f );
```

- `policy`: 执行策略
- `first`：指向要处理的第一个元素的迭代器。
- `last`：指向要处理的最后一个元素的下一个位置的迭代器。
- `f`：一个接受一个参数（迭代器指向的元素）的函数对象或函数指针。函数对象应该返回 `void`，或者返回类型可以被忽略。

### 3种执行策略

C++17 引入了3种执行策略，用于控制标准库算法（如 `std::for_each`）的执行方式：

```c++
#include <iostream>
#include <vector>
#include <algorithm>
#include <execution>

void print_vector(const std::vector<int>& vec) {
    for (int n : vec) {
        std::cout << n << " ";
    }
    std::cout << std::endl;
}

int main() {
    std::vector<int> vec = {1, 2, 3, 4, 5};

    // 顺序执行（单线程）
    std::for_each(std::execution::seq, vec.begin(), vec.end(), [](int& n) { n *= 2; });
    std::cout << "After sequential execution: ";
    print_vector(vec);

    // 并行执行
    std::for_each(std::execution::par, vec.begin(), vec.end(), [](int& n) { n *= 2; });
    std::cout << "After parallel execution: ";
    print_vector(vec);

    // 并行和无序执行,允许对硬件进行最大化优化。
    std::for_each(std::execution::par_unseq, vec.begin(), vec.end(), [](int& n) { n *= 2; });
    std::cout << "After parallel and unsequenced execution: ";
    print_vector(vec);

    return 0;
}

```

## 2. `auto`关键字

相比c++11, auto推导的规则更加直观：

```c++
// c++11 
// 会全都推导为std::initializer_list<int>
auto x3{ 1, 2 }; // std::initializer_list<int>
auto x4 = { 3 }; // decltype(x4) is std::initializer_list<int>
auto x5{ 3 };    // std::initializer_list<int>

// c++17
// 会推导为我们直观认为的
auto x3{ 1, 2 }; // error: not a single element
auto x4 = { 3 }; // decltype(x4) is std::initializer_list<int>
auto x5{ 3 };    // decltype(x5) is int
```

## 3. `namespace嵌套`

```c++
// 以前
namespace A {
    namespace B {
        namespace C {
            void func();
        }
    }
}

// c++17
namespace A::B::C {
    void func();
}
```

## 4. `lambda表达式`

- lambda也是c++11中引入的，在C++11中，lambda表达式只能用捕获this，this是当前对象的一个只读的引用。 
- 在C++17中，可以捕获*this, *this是当前对象的一个拷贝，捕获当前对象的拷贝，能够确保当前对象释放后， lambda表达式能安全的调用this中的变量和方法

## 5. `from_chars`函数和`to_chars`

```c++
// 字符转数字
std::array<char, 3> str{"42"};
int result;
std::from_chars( str.data(), str.data()+str.size(),result );
std::cout << result << std::endl;

// 数字转字符
// p是填充到str以后的最后一个迭代器
if(auto [p, ec] = std::to_chars(str.data(), str.data() + str.size(), 425);
   ec == std::errc()){
    if(p == str.end()){
        std::cout << "hello world\n";
    }
        std::cout << std::string_view(str.data(), p - str.data());
}
输出：
42
hello world
425
```

## 6. 结构化绑定

结构化绑定是指将array、tuple或struct的成员绑定到一组变量*上的语法，最常用的场景是在遍历map/unordered_map时不用再声明一个中间变量了:

```c++
// 以前
for(const auto& kv: map){
  const auto& key = kv.first;
  const auto& value = kv.second;
  // ...
}

// c++17
for(const auto& [key, value]: map){
  // ...
}
```

需要注意的是，结构化绑定的结果并不是变量，c++标准称之为名字/别名，这也导致它们不允许被lambda捕获，但是gcc并没有遵循c++标准，所以以下代码在gcc可以编译，clang则编译不过。

```c++
for(const auto& [key, value]: map){
    [&key, &value]{
        std::cout << key << ": " << value << std::endl;
    }();
}
```

在clang环境下，可以在lambda表达式捕获时显式引入一个引用变量通过编译：

```c++
for(const auto& [key, value]: map){
    [&key = key, &value = value]{
        std::cout << key << ": " << value << std::endl;
    }();
}
```

但是这条限制在c++20中已经被删除，所以在c++20标准中gcc和clang都可以捕获结构化绑定的对象了。上述第一种写法在c++20里都是ok的。

## 7. `std::tuple`的隐式推导

在c++17以前，构造std::pair/std::tuple时必须指定数据类型或使用std::make_pair/std::make_tuple函数

c++17为std::pair/std::tuple新增了推导规则，可以不再显示指定类型。

```c++
// 以前
std::pair<int, std::string> p1{3.14, "pi"s}; // 指定数据类型
auto p1 = std::make_pair(3.14, "pi"s);		 // 或使用std::make_pair/std::make_tuple函数

// c++17
std::pair p3{3.14, "pi"s};					 // 现在直接定义
```

## 8. 编译期判断`if  constexpr`

if constexpr语句是编译期的if判断语句，在C++17以前做编译期的条件判断往往通过复杂SFINAE机制或模版重载实现，甚至嫌麻烦的时候直接放到运行时用if判断，造成性能损耗。

if constexpr大大缓解了这个问题。比如我想实现一个函数将不同类型的输入转化为字符串，在c++17之前需要写三个函数去实现，而c++17只需要一个函数。

- 以前

  ```c++
  // pre c++17
  template <typename T>
  std::string convert(T input){
      return std::to_string(input);
  }
  
  // const char*和string进行特殊处理
  std::string convert(const char* input){
      return input;
  }
  std::string convert(std::string input){
      return input;
  }
  ```

- c++17

  ```c++
  // c++17
  template <typename T>
  std::string convert(T input) {
      if constexpr (std::is_same_v<T, const char*> ||
                    std::is_same_v<T, std::string>) {
          return input;
      } else {
          return std::to_string(input);
      }
  }
  ```

# C++杂记

## 指针，引用和const

参见Tag1中的3，4

- 区别：

  - 引用必须在声明时被初始化，但是不分配存储空间。但指针不在声明时初始化，在初始化的时候需要分配存储空间。
  - 引用初始化后不能被改变，即永远指向某个对象，但指针可以改变所指的对象。
    - 因为指针是一个实体，而引用仅是个别名；
    - 所以 “sizeof 引用”得到的是所指向的变量（对象）的大小，而“sizeof 指针”得到的是指针本身（所指向的变量或对象的**地址**）的大小；
  - 不存在指向空值的引用，但是存在指向空值的指针。
    - 因为引用必须指向一个变量，是这个变量的别名

- 基本使用：

  - 引用：

    ```c++
    int m；  
    int &n = m；  
    ```

    - 引用(reference)n是被引用物(referent)m的别名
    - 对n的任何操作就是对m的操作。
    - **注意这里的&是写在引用n前面的**

  - 指针：

    ```c++
    int  *ip;        // 指针变量的声明
    ip = &var;       // 在指针变量中存储 var 的地址
    ```

    - **注意这里的&是和上面的引用符号不同，它是用来获得变量var的地址的**

- 函数值传递：**值传递、指针传递和引用传递**

  - 值传递：

    ```c++
    void Func1(int x)  
    {  
        x = x + 10;  
    }  
    int n = 0;  
    Func1(n);  
    cout << “n = ” << n << endl;// n = 0 
    ```

    - 改变x不会改变n，n仍然是0

  - 指针传递：

    ```c++
    void Func2(int *x)  
    {  
        (* x) = (* x) + 10;  
    }  
    ⋯  
    int n = 0;  
    Func2(&n);  //将变量n的地址传给函数的指针变量x
    cout << “n = ” << n << endl; // n = 10  
    
    // 指针传递还可以用于传递数组,下面两种效果是一样的
        1. 形式参数是一个指针：
    	void myFunction(int *param)
        2. 形式参数是一个数组：
        void myFunction(int param[])
        //调用时，不用传地址，直接传数组就行：
        int par[];
        myFunction(par);
    ```
  
    - 由于Func2 函数体内的x 是指向外部变量n 的指针，改变该指针的内容将导致n 的值改变，所以n 的值成为10.
  
  - 引用传递：
  
    ```c++
    void Func3(int &x)  
    {  
        x = x + 10;  
    }  
    //...  
    int n = 0;  
    Func3(n);  // x是n的别名，指向同一个东西
    cout << “n = ” << n << endl; // n = 10  
    ```
  
    - 由于Func3 函数体内的x 是外部变量n 的引用，x和n 是同一个东西，改变x 等于改变n，所以n 的值成为10.
    - 能用引用传递就不用指针传递
  
- Const:

  - const只能修饰输入参数,不能修饰输出参数，否则值不变的输出就没有意义了。

    在指针和引用传递时，可以用于防止函数修改传入的参数

    ```c++
    //例如StringCopy 函数：
    void StringCopy(char *strDestination, const char *strSource);
    ```

    - 其中strSource 是输入参数，strDestination 是输出参数。他们都是指针传递，函数内对他们操作，都会改变他们的值
    - 给strSource 加上const修饰后，如果函数体内的语句试图改动strSource 的内容，编译器将指出错误。

  - const还可以修饰函数的返回值

    如果给以“指针传递”方式的函数返回值加const 修饰，那么函数返回值（即指针）的内容不能被修改，该返回值只能被赋给加const 修饰的同类型指针。

    ```c++
    // 例如函数：
    const char * GetString(void);
    // 错误用法：
    char *str = GetString();
    // 正确的用法：
    const char *str = GetString();
    ```

    

## argc和argv

```c++
int main(void);
int main(int argc,char *argv[])// 等于 int main(int argc,char **argv);
```

- **argc**：是argument count 的缩写，保存运行时传递给main函数的参数个数。

- **argv**：是argument vector 的缩写，保存运行时传递main函数的参数，类型是一个字符指针数组，每个元素是一个字符指针，指向一个命令行参数。
  - argv[0]指向程序运行时的全路径名；
  - argv[1] 指向程序在命令行中执行程序名后的第一个字符串；
  - argv[2] 指向程序在命令行中执行程序名后的第二个字符串；
  - 以此类推直到argv[argc], argv[argc] 在C++中指向nullptr，在C语言中指向NULL。

##  静态成员和非静态成员：

- 定义：

  - **静态成员:** 属于整个类，与类的**具体实例**无关。在程序运行之前分配内存，程序结束后释放内存。
  - **非静态成员:** 属于类的**实例**，每个实例拥有一份**独立的副本**。在创建对象时分配内存，对象销毁时释放内存。

- 访问方式:

  - **静态成员:** 通过类名访问，例如 `ClassName::staticMemberName`。
  - **非静态成员:** 通过对象名访问，例如 `objectName.nonStaticMemberName`。

- 生命周期:

  - **静态成员:** 与类同生同死，程序运行期间始终存在。
  - **非静态成员:** 与对象同生同死，创建对象时存在，对象销毁时消失。

- 访问权限:

  - **静态成员:** 可以像普通成员一样设置访问权限（public、private、protected）。
  - **非静态成员:** 通常默认具有私有访问权限，只能在类内部访问。

- 用途:

  - 静态成员:
    - 用于存储与**整个类**相关的信息，例如类名、版本号等。
    - 用于定义**全局函数**或**类工厂函数**，无需创建对象即可调用。
    - 用于定义与**类常量**相关的数据。
  - 非静态成员:
    - 用于存储与**特定对象**相关的信息，例如对象状态、属性等。
    - 用于定义对**特定对象**进行操作的成员函数。

- 例子：

  ```c++
  class Person {
  public:
    static int count; // 静态成员变量，用于统计创建的人数
  
    std::string name; // 非静态成员变量，存储姓名
    int age; // 非静态成员变量，存储年龄
  
    Person(const std::string& name, int age) : name(name), age(age) {
      count++; // 访问静态成员变量
    }
  
    static void printCount() { // 静态成员函数
      std::cout << "Total persons: " << count << std::endl;
    }
  
    void introduce() { // 非静态成员函数
      std::cout << "My name is " << name << " and I am " << age << " years old." << std::endl;
    }
  };
  
  int Person::count = 0; // 静态成员变量的定义
  
  int main() {
    Person p1("Alice", 30);
    Person p2("Bob", 25);
  
    p1.introduce();
    p2.introduce();
  
    Person::printCount(); // 访问静态成员函数
  
    return 0;
  }
  ```

## 关键字：static/inline/explicit

1. **内联（inline）函数**：`inline`关键字用于建议编译器将函数的代码内联插入到调用它的地方，而不是通过函数调用进行执行。这样可以避免函数调用的开销，并且可以在一定程度上提高程序的性能。当函数定义放在头文件中时，使用`inline`关键字可以防止函数重复定义错误。
2. **静态（static）函数**：`static`关键字用于限制函数的作用域。在命名空间或类的内部，使用`static`关键字可以将函数限定为只在当前编译单元或类内部可见。这样可以避免函数与其他编译单元或类中的同名函数冲突，并且可以实现封装。

结合使用`static`和`inline`关键字可以将函数声明为内联且具有静态作用域。这意味着函数的定义将在每个编译单元中内联插入，并且只能在当前编译单元中可见。

然而，编译器是否实际内联函数的代码取决于编译器的实现和优化设置。`inline`关键字只是一种建议，编译器可以选择忽略该建议，并根据自己的优化策略来决定是否内联函数。

3. **explicit**是一个关键字，用于修饰单参数构造函数或转换函数。它的作用是防止编译器进行隐式类型转换。

   当一个构造函数声明为`explicit`时，它将只能被用于显式地创建对象，而不能用于隐式的类型转换。这意味着在使用该构造函数创建对象时，必须使用直接的构造函数调用语法，而不能使用隐式的类型转换。

   比如：

   ```c++
   class MyClass {
   public:
       explicit MyClass(int x) {
           // 构造函数
       }
   };
   
   void func(MyClass obj) {
       // 函数
   }
   
   int main() {
       MyClass obj1(5);  // 正确，直接构造对象
       MyClass obj2 = 10;  // 错误，不能隐式转换
   
       func(obj1);  // 正确，传递对象
       func(10);    // 错误，不能隐式转换
   
       return 0;
   }
   ```

   



## 类里不同成员函数使用同一个类对象

- 问题场景:

  在类A的Init()函数里实例化类B, 然后在类A的update()函数里也想用这个类B的对象

- 先在类A的.h文件里加2个类B/C的指针

  ```c++
  Class A{
  public:
  	LinearTrajectory* traj;
      TrajectoryIteratorCartesian* traj_Car;    
  }
  ```

- 再在A的.cpp文件中实例化

  ```c++
  // traj是指向LinearTrajectory对象的指针
  traj = new LinearTrajectory(initial_pose, end_pose, 0.05,0.5,1.e-3);
  // 由于下面这个类的构造函数的参数是引用,所以需要对指针traj进行解引用操作来获得指针指向的对象,即*traj
  traj_Car = new TrajectoryIteratorCartesian(*traj);
  ```
  
  - 注意上面TrajectoryIteratorCartesian类的定义为`TrajectoryIteratorCartesian(const LinearTrajectory &traj)`
  
    - 这个类的构造函数需要的是类LinearTrajectory的引用.
  
  - 如果TrajectoryIteratorCartesian类的定义为`TrajectoryIteratorCartesian(const LinearTrajectory *traj)`
    
    - 此时就要用`traj_Car = new TrajectoryIteratorCartesian(& traj);`来实例化了
    - 因为此时需要的是一个指针的地址
    

## c++11: 右值引用和移动语义

这两个的目的都是为了避免拷贝操作，用移动资源的方式，降低资源消耗。

### 移动和拷贝的区别

- 移动move:

  移动操作将原始对象的资源转移到新对象中，同时将原始对象置于有效但未指定的状态。移动不会创建资源的副本，而是将资源的所有权转移给新对象。

- 拷贝copy:

  拷贝操作将原始对象的副本创建到新的对象中。新对象与原始对象是独立的，它们之间的修改互不影响。

- 所以移动比拷贝更耗资源。

### 左值和右值的区别

- 左值: 一个占据内存中某个可识别的位置（也就是一个地址）的对象。
- 右值：一个不表示内存中某个可识别位置的对象的表达时

```c++
int x = 10;    // x 是左值, 10是右值
int* p = &x;   // 正确，可以取左值x的地址
int* p = &10;  // 错误，右值10不能取地址
```

### 右值引用`&&`

如上面的例子所是，传统的左值引用`&`只能绑定到的左值。但`&&`可以绑定到右值，让我们可以临时的窃取一个对象的资源。

直接引用右值就可以避免不必要的深拷贝，减少资源的分配和释放，提高代码的执行效率。

右值引用主要用于**类对象**，以优化对象的转移和避免不必要的拷贝。

例子：

```c++
class MyClass {
public:
    MyClass() : data(new int[1000]) {}

    // 移动构造函数
    MyClass(MyClass&& other) noexcept : data(other.data) {
        other.data = nullptr;
    }

    // 移动赋值运算符
    // 在构造新的实例时，把另一个实例other的资源移动给新的实例
    MyClass& operator=(MyClass&& other) noexcept {
        if (this != &other) {
            delete[] data;
            data = other.data;
            other.data = nullptr;
        }
        return *this;
    }

    ~MyClass() {
        delete[] data;
    }

private:
    int* data;
};

```

### 移动语义`std::move`

`std::move` 是标准库提供的一个函数模板，用于将左值强制转换为右值引用。这使得我们能够显式地指定对象可以被“移动”，从而启用移动语义。

两个例子：

```c++
// 1. 显式移动
std::vector<int> v1 = {1, 2, 3};
std::vector<int> v2 = std::move(v1); // v1 现在为空，资源被转移到 v2

// 2. 避免不必要拷贝
MyClass createObject() {
    MyClass obj;
    // 使用 std::move 将临时对象转换为右值引用
    return std::move(obj);
}
```



## c++11: 智能指针

### 什么是智能指针

- 智能指针（Smart Pointer）是一种用于管理动态分配的资源（通常是内存）的 C++ 技术，旨在**简化资源的生命周期管理**，从而**减少内存泄漏和资源管理错误的风险**。
- 智能指针是一种封装了原始指针的c++类，它们提供了自动化资源管理的功能，**避免了手动释放内存或资源的繁琐工作**。
- 智能指针是在 [<memory>](https://learn.microsoft.com/zh-cn/cpp/standard-library/memory?view=msvc-170) 头文件中的 `std` 命名空间中定义的
- 主要目标：
  - **自动化资源释放：** 智能指针会在适当的时候自动释放其所管理的资源，从而避免内存泄漏和资源泄漏。
  - **防止悬挂指针：** 通过合适的生命周期管理，智能指针可以防止悬挂指针（指向已释放内存的指针）的情况。
  - **提高代码安全性：** 由于智能指针负责管理资源，可以减少人为错误，从而提高代码的安全性和可维护性。

- 三种智能指针类型：
  - `std::unique_ptr`：独占所有权的智能指针，用于确保在特定时间只有一个指针指向资源。
  - `std::shared_ptr`：允许多个指针共享资源所有权的智能指针，使用引用计数来管理资源的生命周期。
  - `std::weak_ptr`：用于避免循环引用和解决 `std::shared_ptr` 可能的资源泄漏问题。

### 和传统指针的对比

```c++
// 传统指针：
void UseRawPointer()
{
    // Using a raw pointer -- not recommended.
    Song* pSong = new Song(L"Nothing on You", L"Bruno Mars"); 

    // Use pSong...

    // Don't forget to delete!
    delete pSong;   
}


// 智能指针
void UseSmartPointer()
{
    // Declare a smart pointer on stack and pass it the raw pointer.
    unique_ptr<Song> song2(new Song(L"Nothing on You", L"Bruno Mars"));

    // Use song2...
    wstring s = song2->duration_;
    //...

} // song2 is deleted automatically here.
```



### 使用例子

演示了如何使用 C++ 标准库中的 `unique_ptr` 智能指针类型将指针封装到大型对象。

```c++
class LargeObject
{
public:
    void DoSomething(){}
};

void ProcessLargeObject(const LargeObject& lo){}
void SmartPointerDemo()
{    
    // 将智能指针声明为一个自动（局部）变量pLarge。 
    // 不要对智能指针本身使用 new 或 malloc 表达式。
    // 在类型参数中即<>内，指定封装指针的指向类型：<LargeObject>
    // 在智能指针构造函数中将原始指针传递至 new 对象：(new LargeObject())
    // Create the object and pass it to a smart pointer
    std::unique_ptr<LargeObject> pLarge(new LargeObject());

    // 智能指针使用重载的 -> 和 * 运算符访问对象
    //Call a method on the object
    pLarge->DoSomething();
    // Pass a reference to a method.
    ProcessLargeObject(*pLarge);

} //pLarge is deleted automatically when function block goes out of scope.

void SmartPointerDemo2()
{
    // Create the object and pass it to a smart pointer
    std::unique_ptr<LargeObject> pLarge(new LargeObject());

    //Call a method on the object
    pLarge->DoSomething();
	
   	// 允许智能指针删除对象
    // Free the memory before we exit function block.
    pLarge.reset();

    // Do some other work...
}
```



## 创建新对象make_unique/new/malloc

`std::make_unique`、`new` 和 `malloc` 都是用于分配内存的机制

- **std::make_unique**:

  - 是 C++11 引入的标准库函数，用于创建动态分配对象的 `std::unique_ptr` 智能指针。
  - **自动管理内存**，不需要手动释放。
  - 支持在创建对象时传递构造函数参数。
  - 更安全，能够防止资源泄漏和内存管理错误。

  ```c++
  // 使用 std::make_unique 创建动态分配的整数对象
  std::unique_ptr<int> ptr = std::make_unique<int>(42);
  
  // ptr 离开作用域时，分配的整数对象会被自动释放
  ```

- **new**

  - 是 C++ 中的运算符，用于在堆上分配内存，并返回指向分配内存的指针。
  - 需要手动释放内存，使用 `delete` 运算符来释放。
  - 支持在分配内存时调用构造函数。
  - **可以用于任何类型的对象**。
  - 可能会出现内存泄漏和悬挂指针问题，需要谨慎管理。

  ```c++
  // 使用 new 运算符分配整数对象的内存
  int* ptr = new int(42);
  
  // 使用完毕后，需要手动释放内存
  delete ptr;
  ```

- **malloc**

  - 是 C 语言中的函数，用于分配内存，并返回指向分配内存的指针。
  - 需要手动释放内存，使用 `free` 函数来释放。
  - **不会调用对象的构造函数**，只是简单分配内存空间。即不能用于类
  - 通常用于分配基本数据类型或无需构造函数初始化的数据。
  - 与 C++ 的对象管理机制不兼容，可能导致资源泄漏和内存管理错误。

  ```c++
  // 使用 malloc 函数分配整数对象的内存
  int* ptr = (int*)malloc(sizeof(int));
  
  // 使用完毕后，需要手动释放内存
  free(ptr);
  ```

## c++11: lambda表达式(匿名函数)

- **什么是c++的lambda表达式**

  C++ 中的 lambda 表达式是一种匿名函数，它允许你在需要函数的地方定义一个短小的函数体，而不必显式地声明一个命名函数

- **lambda表达式的形式**
  
  ```
  [capture_clause](parameters) -> return_type {function_body}
  ```
  
  -  capture_clause：用于捕获外部变量，决定了 lambda 表达式是否能够访问和修改外部作用域的变量。以为空、捕获所有外部变量的引用 [&]，或捕获所有外部变量的值 [=] 等。
  - parameters： 类似于函数参数，指定了 lambda 表达式的参数列表。
  - return_type：指定 lambda 表达式的返回类型。
  - function_body：实现具体的功能代码。
  
- **lambda表达式相比直接定义函数的优势**

  1. **减少代码量：** 在某些简单的场景下，使用 lambda 表达式可以避免定义独立的函数，从而减少代码的冗余，使代码更为紧凑。
  2. **局部性：** lambda 表达式在定义它们的地方局部可见，不会污染全局命名空间，使得代码更加清晰。
  3. **避免命名冲突：** lambda 表达式没有命名，不需要担心与已有的函数或变量名冲突。
  4. **内联使用：** lambda 表达式可以在需要函数的地方直接使用，无需单独声明和定义，使得代码更加内联。
  5. **方便作为参数：** lambda 表达式可以作为函数的参数传递，用作回调函数，避免了定义额外的命名函数。
  6. **捕获外部变量：** lambda 表达式可以捕获外部作用域的变量，使其在 lambda 表达式内部可用，提供了更大的灵活性。
  7. **短时使用：** 当只需要一个临时函数，而不想为其分配额外的命名时，lambda 表达式非常方便。

- **例子**

  ```c++
  // 例1：从外部捕获变量x的引用，并返回一个double类型的值
  int x = 10;
  auto calculate = [&x](double a, double b) -> double {
      x += a*b;
      return x;
  };
  double result = calculate(2.5, 3.0); // 返回值为17.5
  
  
  // 例2：从外不捕获所有的变量的值，对他们进行计算
  int y = 20;
  int x = 5;
  auto modify = [=]() {
      y += x;
  };
  modify();
  // 因为这里用"="捕获的是外部变量的值不是"&"，所以lambda表达式内的操作不会影响外部变量y的值，y还是20 
  std::cout << y << std::endl;  // 输出 20
  
  ```
  

## 确保类只有一个实例

在C++中，将**构造函数私有化**是一种实现单例模式（Singleton Pattern）的常见手段之一。

lsd-slam中内存管理就用到了这个技术，参见lsd_slam_core/DataStructures/FrameMemory.h.

下面是一个示例：

```c++
class Singleton {
private:
    // 将构造函数私有化
    Singleton() {}

public:
    // 静态成员函数，用于获取单例对象
    static Singleton& getInstance() {
        static Singleton instance;  // 保证只创建一个实例
        return instance;
    }

    // 其他成员函数和数据
    // ...
};

int main() {
    // 错误示例：试图直接调用构造函数
    // Singleton obj;  // 编译错误，构造函数是私有的

    // 正确示例：通过静态成员函数获取单例对象
    Singleton& singleton = Singleton::getInstance();
    // 使用 singleton 对象进行操作
    // ...

    return 0;
}
```



##  友元函数和友元类

作用：为了在类的成员函数外部直接访问对象的**私有成员**。友元friend，相当于是说：朋友是值得信任的，所以可以对他们公开一些自己的隐私。

> 类的私有变量只能被同一类的成员函数访问

- **友元函数**

  在定义一个类的时候，可以把一些函数（包括全局函数和其他类的成员函数）声明为“友元”，这样那些函数就成为该类的友元函数，在友元函数内部就可以访问该类对象的私有成员了。

  - 写法：

    ```c++
    // 将全局函数声明为友元的写法如下：
    friend  返回值类型  函数名(参数表);
    
    // 将其他类的成员函数声明为友元的写法如下：
    friend  返回值类型  其他类的类名::成员函数名(参数表);
    ```
    
  - 实例

    ```c++
    #include<iostream>
    using namespace std;
    
    // 提前声明CCar类，以便后面的CDriver类使用
    // 直接定义CCar类，是无法解决这个问题的，因为CCar类中需要用到CDriver类。因此，只能提前声明其中一个类
    class CCar;  
    
    class CDriver
    {
    public:
        void ModifyCar(CCar* pCar);  //改装汽车
    };
    
    class CCar
    {
    private:
        int price;
        //声明为友元函数，所以作为非类函数的MostExpensiveCar()也能访问类的私有变量了
        friend int MostExpensiveCar(CCar cars[], int total);  
        //声明为友元函数，所以作为另一个类的函数ModifyCar()也能访问类的私有变量了
        friend void CDriver::ModifyCar(CCar* pCar);  
    };
    
    void CDriver::ModifyCar(CCar* pCar)
    {
        pCar->price += 1000;  //汽车改装后价值增加
    }
    
    int MostExpensiveCar(CCar cars[], int total)  //求最贵气车的价格
    {
        int tmpMax = -1;
        for (int i = 0; i<total; ++i)
            if (cars[i].price > tmpMax)
                tmpMax = cars[i].price;
        return tmpMax;
    }
    ```

- **友元类**

  一个类 A 可以将另一个类 B 声明为自己的友元，类 B 的所有成员函数就都可以访问类 A 对象的私有成员

  - 写法：

    ```c++
    friend  class  类名;
    ```

  - 示例

    ```c++
    class CCar
    {
    private:
        int price;
        //声明 CDriver 为自己的友元类，那么CDriver的所有函数都可以访问自己的私有变量
        friend class CDriver;  
    };
    
    class CDriver
    {
    public:
        CCar myCar;
        void ModifyCar()  //改装汽车
        {
            myCar.price += 1000;  //因CDriver是CCar的友元类，故此处可以访问其私有成员
        }
    };
    int main()
    {
        return 0;
    }
    ```

##  模板类/函数/结构体

- 为什么用模板？

  当你发现一套操作对多个不同类型的变量操作时，为了避免重复定义多个类/函数/结构体却只是变一变数据类型，我们可以使用模板。

### 函数模板

体现在：调用函数时传递的参数类型

```c++
// ---------- 语法：----------
template<class 数据类型参数标识符>
<返回类型><函数名>(参数表)
{
    函数体
}

// ---------- 示例：----------
//下面这个函数就可以遍历输出各个数据类型的数组元素
template <class T>  //定义函数模板
void outputArray(const T *array, int count) {
    for (int i = 0; i < count; i++)
        cout << array[i] << " "; //如果数组元素是类的对象，需要该对象所属类重载了流插入运算符“<<”
    cout << endl;
}
```

### 结构体模板

体现在：声明结构元素时 StackNode<类型> s

```c++
// ---------- 示例：----------
template<class T>
struct StackNode
{
　　struct T data;
　　struct StackNode<T> *next;
};

```

### 类模板

体现在:声明类对象时 Stack<类型> s

```c++
// ---------- 示例：----------
template<class T>
class Stack
{
　public:
　　T pop();
　　bool push(T e);
　private:
　　StackNode<T> *p;
}
template<class T>//类模板外的 成员函数实现
T Stack<T>::pop()
{...}
```

##  宏

宏（Macro）是C/C++预处理器的一部分，它允许在代码中定义常量、简单的代码块替换、参数化代码替换等。宏在代码编译前会由预处理器进行展开和替换。

### 用宏的优点：

1. **代码生成**：宏允许在编译时进行代码生成，因此可以用来生成重复的代码或模板。

   如下面带参数的例子，都不需要定义参数的数据类型，就保证了对不同数据类型实现相同操作

2. **条件编译**：宏可以用于条件编译，根据编译器选项或其他条件选择性地包含或排除代码块。这对于实现跨平台代码或处理不同编译器的特定行为非常有用。

3. **性能优化**：由于宏展开是在预处理阶段完成的，因此它们可以比函数调用更快，特别是对于简单的操作。这对于性能敏感的应用程序非常重要。

### 什么时候需要宏

- **定义常量**：需要在多个地方使用的常量值，例如数学常数、物理常数等。
- **简化重复代码**：有大量重复代码时，可以通过宏定义来简化。
- **条件编译**：需要根据不同的平台、编译器或配置编译不同的代码时。
- **代码参数化**：需要对一些简单的代码进行参数化处理时，例如数学运算等。

### 简单例子：

- 常量替换

  ```
  #define PI 3.14159
  ```

- 代码块替换

  ```
  #define PRINT_HELLO std::cout << "Hello, World!" << std::endl;
  ```

- 带参数的代码块替换

  ```
  #define SQUARE(x) ((x) * (x))
  ```

### 复杂例子：

宏要写在一行内，但如果很复杂就可以用`\`来换行增加代码的可阅读性。

```c++
// MTK: Matrix toolkit?
#define MTK_BUILD_MANIFOLD(name, entries) \
struct name { \
	typedef name self; \
    // 成员变量定义
	std::vector<std::pair<int, int> > S2_state;\
	std::vector<std::pair<int, int> > SO3_state;\
	std::vector<std::pair<std::pair<int, int>, int> > vect_state;\
    // 这里又是一个宏
	MTK_SUBVARLIST(entries, S2_state, SO3_state) \
    // 构造函数定义
	name ( \
		MTK_TRANSFORM_COMMA(MTK_CONSTRUCTOR_ARG, entries) \
		) : \
		MTK_TRANSFORM_COMMA(MTK_CONSTRUCTOR_COPY, entries) {}\
	// 成员函数定义
    int getDOF() const { return DOF; } \
	void boxplus(const MTK::vectview<const scalar, DOF> & __vec, scalar __scale = 1 ) { \
		MTK_TRANSFORM(MTK_BOXPLUS, entries)\
	} \
	void oplus(const MTK::vectview<const scalar, DIM> & __vec, scalar __scale = 1 ) { \
		MTK_TRANSFORM(MTK_OPLUS, entries)\
	} \
	void boxminus(MTK::vectview<scalar,DOF> __res, const name& __oth) const { \
		MTK_TRANSFORM(MTK_BOXMINUS, entries)\
	} \
	friend std::ostream& operator<<(std::ostream& __os, const name& __var){ \
		return __os MTK_TRANSFORM(MTK_OSTREAM, entries); \
	} \
	void build_S2_state(){\
		MTK_TRANSFORM(MTK_S2_state, entries)\
	}\
	void build_vect_state(){\
		MTK_TRANSFORM(MTK_vect_state, entries)\
	}\
	void build_SO3_state(){\
		MTK_TRANSFORM(MTK_SO3_state, entries)\
	}\
	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res, int idx) {\
		MTK_TRANSFORM(MTK_S2_hat, entries)\
	}\
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res, int idx) {\
		MTK_TRANSFORM(MTK_S2_Nx_yy, entries)\
	}\
	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, Eigen::Matrix<scalar, 2, 1> dx, int idx) {\
		MTK_TRANSFORM(MTK_S2_Mx, entries)\
	}\
	friend std::istream& operator>>(std::istream& __is, name& __var){ \
		return __is MTK_TRANSFORM(MTK_ISTREAM, entries); \
	} \
};
```

## 回调函数

一个函数可以作为另一个函数的参数。这种技术通常称为**回调函数**。在C++中，可以通过函数指针或函数对象（如lambda表达式、`std::function`）来实现。

回调函数的目的就是为了将变化的模块抽离出来，延迟实现；固定的模块抽象出来设计好，“写死”。

### 两种实现的区别

- 函数指针是一个指向函数的指针，它可以用于调用该函数。函数指针通常声明为指向特定类型和特定参数列表的函数
  - 优点：简单易用，速度快
  - 缺点：
    1. 不能够处理可调用对象，例如lambda表达式、函数对象、成员函数等
    2. 函数指针只能指向特定类型和特定参数列表的函数，如果需要切换到不同类型或者不同参数列表的函数，就需要修改函数指针的类型。
- std::function是c++11为了解决函数指针的缺陷而引入的
  - 它可以存储任意可调用对象，包括函数指针、函数对象、成员函数和lambda表达式等。
  - 运行时多态：可以在运行时确定其类型和参数列表。

### 函数指针实现

```c++
#include <iostream>
// ===========不变的模块，写死================
// 定义一个函数指针
typedef void (*CallbackFunc)(int);

// 接受函数指针即回调函数作为参数
void processData(int data, CallbackFunc callback) {
    // 执行一些处理
    data *= 2;
    // 调用回调函数
    callback(data);
}

// =========变化的模块，放到客户端进行实现========
// 回调函数的实现
void myCallback(int value) {
    std::cout << "Callback called with value: " << value << std::endl;
}

int main() {
    // 调用带有回调函数的函数
    processData(10, myCallback);
    return 0;
}

```

### std::function实现

```c++
#include <iostream>
#include <functional>
// ===========不变的模块，写死================
// 接受 std::function 作为参数的函数
void processData(int data, std::function<void(int)> callback) {
    // 执行一些处理
    data *= 2;
    // 调用回调函数
    callback(data);
}

// =========变化的模块，放到客户端进行实现========
// 函数
void myCallback(int value) {
    std::cout << "Callback called with value: " << value << std::endl;
}
// lambda表达式(匿名函数)
auto lambda = [](int value) {
    std::cout << "Lambda callback called with value: " << value << std::endl;
}

int main() {
    // **************** std::function可以接收任何可调用的对象 ****************
    // 1. 接收一个函数
    processData(10, myCallback);
    
    // 2. 接收一个lambda表达式
    processData(lambda(10));

    return 0;
}

```

### std::function理解

- 基本概念

  `std::function` 本身是一个模板类，用于存储和调用可调用对象（函数、lambda 表达式、函数对象等）。其返回值取决于所存储的可调用对象的返回类型。

  就相当于创建了一个类，类提供了一个接口，这个接口根据数据类型来选择函数？

- 常见用途：

  **回调函数**：`std::function` 常用于设置回调函数，例如事件处理、异步操作完成后的处理等。

  **函数参数**：可以作为函数参数，允许函数接受和调用用户自定义的可调用对象。

- 一个嵌套在类里例子

  高博激光slam书中的`io_utils.h` 和`run_imu_integration.cc`
  
  ```c++
  // ===========不变的模块，写死================:: io_utils.h
  class TxtIO {
     public:
      //  构造函数使用ifstream直接读取文件
      TxtIO(const std::string &file_path) : fin(file_path) {}
  
      // 1. 定义回调函数类型
      // 因为有种数据，所以定义三个std::function类型
      using IMUProcessFuncType = std::function<void(const IMU &)>;
      using OdomProcessFuncType = std::function<void(const Odom &)>;
      using GNSSProcessFuncType = std::function<void(const GNSS &)>;
  
      // 2. 注册回调函数
  	// 定义三种接受 std::function 作为参数的函数
      TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
          imu_proc_ = std::move(imu_proc);	// 避免深拷贝，节省资源
          return *this;
      }
  
      TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc) {
          odom_proc_ = std::move(odom_proc);
          return *this;
      }
  
      TxtIO &SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
          gnss_proc_ = std::move(gnss_proc);
          return *this;
      }
  
      // 3. 在Go()中遍历文件内容，调用传递进来回调函数  
      void Go();
  
     private:
      std::ifstream fin;
      IMUProcessFuncType imu_proc_;
      OdomProcessFuncType odom_proc_;
      GNSSProcessFuncType gnss_proc_;
  };
  
  // =========变化的模块，放到客户端进行实现========:: run_imu_integration.cc
  sad::TxtIO io(FLAGS_imu_txt_path);
  // 这里传了一个lambda表达式进去
  // 这个匿名函数的参数imu，在Go()中调用回调函数imu_proc_时被赋予
  io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui](const sad::IMU& imu) { xxx
  }).Go();
  
  
  // 附上Go()的实现
  // ========= 不变的模块，写死 =========:: io_utils.cc
  void TxtIO::Go() {
      if (!fin) {
          LOG(ERROR) << "未能找到文件";
          return;
      }
  
      while (!fin.eof()) {
          std::string line;
          std::getline(fin, line);
          if (line.empty()) {
              continue;
          }
  
          if (line[0] == '#') {
              // 以#开头的是注释
              continue;
          }
  
          // load data from line
          std::stringstream ss;
          ss << line;
          std::string data_type;
          ss >> data_type;
  
          if (data_type == "IMU" && imu_proc_) {
              double time, gx, gy, gz, ax, ay, az;
              ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
              // imu_proc_(IMU(time, Vec3d(gx, gy, gz) * math::kDEG2RAD, Vec3d(ax, ay, az)));
              imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
          } else if (data_type == "ODOM" && odom_proc_) {
              double time, wl, wr;
              ss >> time >> wl >> wr;
              odom_proc_(Odom(time, wl, wr));
          } else if (data_type == "GNSS" && gnss_proc_) {
              double time, lat, lon, alt, heading;
              bool heading_valid;
              ss >> time >> lat >> lon >> alt >> heading >> heading_valid;
              gnss_proc_(GNSS(time, 4, Vec3d(lat, lon, alt), heading, heading_valid));
          }
      }
  
      LOG(INFO) << "done.";
  }
  ```
  

## 信号

信号是软件中断，它是在软件层次上对中断机制的一种模拟，是一种异步通信的方式（异步：不知道信号什么时候会来）。信号可以导致一个正在运行的进程被另一个正在运行的异步进程中断，转而处理某一个突发事件。这里我们学习的“信号”就是属于这么一种“中断”。我们在终端上敲“Ctrl+c”，就产生一个“中断”，相当于产生一个信号，接着就会处理这么一个“中断任务”（默认的处理方式为中断当前进程）。

> 中断：先停下本进程，去处理信号。内核进程可以利用它来通知用户空间进程发生了哪些系统事件。 

### 信号要素

每个信号必备4要素，分别是：1.编号 2.名称 3.事件 4.默认处理动作。

参考[网页](https://www.cnblogs.com/codingbigdog/p/16246557.html)

### 信号阻塞sigprocmask()

- 有时候不希望在接到信号时就立即停止当前执行，去处理信号，同时也不希望忽略该信号，而是延时一段时间去调用信号处理函数。这种情况是通过阻塞信号实现的。

- 信号递达（Delivery）：执行信号的处理动作

  信号未决（Pending）：信号从产生到递达之间的状态

  - 进程可以选择阻塞（Block）某个信号。被阻塞的信号产生时将保持在未决状态，直到进程解除对此信号的阻塞，才执行递达的动作。

- sigprocmask()

  ```c
  #include <signal.h>      
  int sigprocmask(ubt how,const sigset_t*set,sigset_t *oldset); 
  ```

  **参数：**

  - how：用于指定信号修改的方式，可能选择有三种
    - SIG_BLOCK			
      - 将set所指向的信号集中包含的信号加到当前的信号掩码中。即信号掩码和set信号集进行或操作。
    - SIG_UNBLOCK
      - 将set所指向的信号集中包含的信号从当前的信号掩码中删除。即信号掩码和set进行与操作
    - SIG_SETMASK
      - 将set的值设定为新的进程信号掩码。即set对信号掩码进行了赋值操作
  - set：为指向信号集的指针，在此专指新设的信号集，如果仅想读取现在的屏蔽值，可将其置为NULL。
  - oldset：也是指向信号集的指针，在此存放原来的信号集。可用来检测信号掩码中存在什么信号。

  **返回：**

  - 成功执行时，返回0。失败返回-1，errno被设为EINVAL

- sigsuspend()

  ```c
  int sigsuspend(const sigset_t*sigmask);
  ```

  **进程执行到sigsuspend时，sigsuspend并不会立刻返回，进程处于TASK_INTERRUPTIBLE状态并立刻放弃CPU，等待UNBLOCK（mask之外的）信号的唤醒** 。进程在接收到UNBLOCK（mask之外）信号后，调用处理函数，然后把现在的信号集还原为原来的，sigsuspend返回，进程恢复执行。

- 例子：

  ```c
  #include <unistd.h>
  #include <signal.h>
  #include <stdio.h>
  void handler(int sig)   //信号处理程序
  {
     if(sig == SIGINT)
        printf("SIGINT sig");
     else if(sig == SIGQUIT)
        printf("SIGQUIT sig");
     else
        printf("SIGUSR1 sig");
  }
   
  int main()
  {
      sigset_t new,old,wait;   //三个信号集
      struct sigaction act;
      act.sa_handler = handler;
      sigemptyset(&act.sa_mask);
      act.sa_flags = 0;
      sigaction(SIGINT, &act, 0);    //可以捕捉以下三个信号：SIGINT/SIGQUIT/SIGUSR1
      sigaction(SIGQUIT, &act, 0);
      sigaction(SIGUSR1, &act, 0);
     
      sigemptyset(&new);
      sigaddset(&new, SIGINT);  //SIGINT信号加入到new信号集中
      sigemptyset(&wait);
      sigaddset(&wait, SIGUSR1);  //SIGUSR1信号加入wait
      sigprocmask(SIG_BLOCK, &new, &old);       //将SIGINT阻塞，保存当前信号集到old中
     
      //临界区代码执行    
    
      if(sigsuspend(&wait) != -1)  //程序在此处挂起；用wait信号集替换new信号集。即：过来SIGUSR1信  号，阻塞掉，程序继续挂起；过来其他信号，例如SIGINT，则会唤醒程序。执行sigsuspend的原子操作。注意：如果“sigaddset(&wait, SIGUSR1);”这句没有，则此处不会阻塞任何信号，即过来任何信号均会唤醒程序。
          printf("sigsuspend error");
      printf("After sigsuspend");
      sigprocmask(SIG_SETMASK, &old, NULL);
      return 0;
  }
  ```

  

# C++小功能

## 1. 按空格分割字符串

1. 直接用stringsream来直接分割

   ```c++
   int main(){
       ifstream f;
       f.open("/home/yang/Desktop/testeverything/c++/normal_test/test.txt");
       
       while (!f.eof())
       {
           // 可以是从文件里读的，也可以是直接一个字符串，都可以直接分割
           string s = "1311867170.462290 rgb/1311867170.462290.png";
           // getline(f,s);
           // cout << s << endl;
           if(!s.empty()){
               stringstream ss;
               ss <<s;
               double t;
               string sRGB;
               ss >> t;
               ss >> sRGB;
               cout << t << endl << sRGB << endl;
           }
       }
   }
   ```

   

## 2. 读取json

nlohmann的教程：

[git](https://github.com/nlohmann/json?tab=readme-ov-file)上也有详细使用教程

https://www.cnblogs.com/linuxAndMcu/p/14503341.html

### 下载使用nlohmann/json

从git上将[nlohmann目录](https://github.com/nlohmann/json/tree/develop/include)拷贝到新建工程目录的include目录下，然后在c++代码中加入头文件就可以用了

```c++
#include "nlohmann/json.hpp"
// 创建空的 JSON 数组
nlohmann::json json_array = nlohmann::json::array();
```

### 一个例子：读取并输出所有键值

- 目录结构：

  ```
  .
  ├── build
  │   ├── ...
  │   └── test
  ├── CMakeLists.txt
  ├── files
  │   └── test.json
  ├── include
  │   └── nlohmann
  └── src
      └── test.cpp
  ```

- CMakeLists.txt

  ```cmake
  cmake_minimum_required(VERSION 2.8)
  
  project(test)
  include_directories(${PROJECT_SOURCE_DIR}/include)
  add_executable(test src/test.cpp)
  ```

- test.json

  ```json
  {
    "output": {
      "width": 720,
      "height": 1080,
      "frameRate": 20,
      "crf": 31
    },
    "tracks": [
      {
        "name": "t1",
        "pieces": [
          {
            "file": "x.mp4",
            "startTime": 2,
            "endTime": 6
          },
          {
            "file": "y.mp4",
            "startTime": 9,
            "endTime": 13
          }
        ]
      },
      {
        "name": "t2",
        "pieces": [
          {
            "file": "z.mp4",
            "startTime": 0,
            "endTime": 10
          }
        ]
      }
    ]
  }
  ```

- test.cpp

  ```c++
  #include "nlohmann/json.hpp"
  #include <fstream>
  #include <iostream>
  
  int main() {
      // 打开json文件
      std::string fileName = "../files/test.json";
      std::ifstream jfile(fileName);
      // 读取该json文件信息
      nlohmann::json jsonData;
      jfile >> jsonData;
  
      // 打印output对象【也可以用j["output"].at("width")】
      std::cout << jsonData.at("output").at("width") << std::endl;
      std::cout << jsonData.at("output").at("height") << std::endl;
      std::cout << jsonData.at("output").at("frameRate") << std::endl;
      std::cout << jsonData.at("output").at("crf") << std::endl;
      // 打印tracks数组对象
      for(int i=0; i<jsonData["tracks"].size(); i++) {
          std::cout << jsonData["tracks"][i].at("name") << std::endl;
  
          // 打印pieces子数组对象
          nlohmann::json jsonData2 = jsonData["tracks"][i].at("pieces");
          for(int k=0; k<jsonData2.size(); k++) {
              std::cout << jsonData2[k].at("file") << std::endl;
              std::cout << jsonData2[k].at("startTime") << std::endl;
              std::cout << jsonData2[k].at("endTime") << std::endl;
          }
      }
  
      return 0;
  }
  ```

  

### 保存数组

```json
"extrinsic_param": {
        "rotation": [
            0.07842524008213057,
            0.11624746577306232,
            -1.7437338500913855
        ],
```

使用vector来保存：

```c++
nlohmann::json jsonData2 = jsonData["lidars"][i].at("extrinsic_param");
std::vector<double> pose;
for(auto& element: jsonData2.at("rotation")) {
    pose.push_back(element);
}
```

