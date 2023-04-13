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

# C++杂记
   ## 1. 指针，引用和const

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
    	void myFunction(int *param{}
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

    

## 2. argc和argv

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

## 3. 模板类/函数/结构体

- 为什么用模板？

  当你发现一套操作对多个不同类型的变量操作时，为了避免重复定义多个类/函数/结构体却只是变一变数据类型，我们可以使用模板。

- 函数模板

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

- 结构体模板

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

- 类模板

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

  