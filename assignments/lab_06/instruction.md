# Laboratorní úloha číslo 6 - Plánování tras
Cílem tohoto cvičení je implementovat algoritmus pro plánování trasy mezi dvěma souřadnicemi, a to konkrétně pomocí algoritmu A* (A-star). Studenti se naučí pracovat s mapou jako s grafem a vyhledávat v něm optimální řešení. Součástí cvičení je realizace služby (service), která bude dostupná klientům v jiných nodech, a která bude řešit problematiku plánování. Základní plán trasy bude následně rozšířen tak, aby byl dodržen bezpečný odstup od překážek, a aby byla trasa vyhlazena.
V rámci domácí přípravy studenti pochopí obecnou strukturu deterministických plánovacích algoritmů a budou vědět, které programové nástroje využijí k implementaci.

## Cíl cvičení 
Výledkem cvičení je:
  1) Služba `/plan_path`, která bude vracet naplánovanou trasu.
  2) Publikování naplánované trasy na topicu `/planned_path`.
  3) Vizualizace a ověření dat v Rvizu.

### Ukázka možného řešení
![AfterLocalization](.fig/astar.png)

## Domácí příprava

> [!WARNING]  
> Zadání je spíše časově náročné, minimální nutná podmína je teoretické pochopení algoritmu a základní znalost práce se systémem ROS. Doporučuji ale si řešení částečně připravit doma. 

> [!CAUTION]
> Na konci cvičení bude práce ohodnocena až **5 body**!

### Pochopení plánovacích algoritmů
Cílem je na základě mapy prostředí a dvou pozic najít optimální trasu. Nastudujte si princip plánovacích algoritmů, s důrazem na algoritmus A*.

- Jak je reprezentovaná mapa a kde ji v rámci simulátoru můžeme získat?
- Jak převést souřadnice ze světového souřadnicového systému do systému mapy a zpět?
- Co znamenají pojmy "cost funkce" a "heuristická funkce"?
- Jaký význam má prioritní fronta a dle jakého kritéria ji budete řadit?
- Jaké informace o jednotlivých buňkách mapy je nutné během plánování uchovávat?

### Práce s ROS 2
Základy práce s ROS 2 jste se naučili v rámci minulého cvičení. V tomto týdnu přidáme práci s ROS services (služby), jedná se o request-based způsob komunikace v ROSu. 
- Jak zaslat a obsloužit požadavek na službu v ROS 2 za použití rclcpp::Node?
- Seznamte se se strukturou použitých zpráv a služeb z `nav\_msgs`.

### Bezpečný odstup od překážek
Algoritmus A* ve své základní podobě vyhledává nejkratší trasu, která často vede podél hrany překážek, a pokud bychom navigovali střed robotu o nenulových rozměrech podél takové trasy, došlo by jistě ke kolizi. Obecně je lepší udržovat spíše větší odstup od překážek z důvodu nejistoty lokalizace robotu.

Jeden z možných přístupů je tzv. morfologická dilatace překážek (jak dilatace funguje si můžete přečíst na této [stránce](https://homepages.inf.ed.ac.uk/rbf/HIPR2/dilate.htm)). V podstatě by mělo dojít k "nafouknutí" překážek. Tento přístup není vhodný např. ve scénářích s úzkými koridory, ale pro účely našeho světa v simulátoru je to řešení dostačující.

### Vyhlazení trasy
Trasy vygenerované algoritmem A* jsou z principu zarovnané s mřížkou mapy, ve které plánování probíhá. To není vždy optimální, zejména s ohledem na kinematiku a dynamiku robotu s diferenciálním podvzokem. Vyplatí se proto trasu upravit tak, aby obsahovala minimum "ostrých zatáček", tj. posunout jednotlivé waypointy trasy tak, aby výsledek byl maximálně plynulý (matematicky vyjádřeno, pokud by trasa byla popsána spojitou funkcí, pak i její první deriace by měla být spojitá). 

Existuje více metod vedoucích k podobnému cíli, jako jednu variantu můžu doporučit iterativní gradientní algoritmus, který si můžet nastudovat na tomto [odkazu](https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4).


## Hodnocení cvičení
> [!WARNING]  
> Cvičení nebude hodnoceno jen na základě funkčnosti. Bude hodnocen i zdrojový kód a pochopení problematiky. Doporučuji se připravit na otázky z domácí přípravy. 

V projektu mpc\_rbt\_student budete upravovat soubor `Planning.cpp` a příslušný hlavičkový soubor. Pro kompilaci programu budete upravovat `CMakeLists.txt` a `package.xml`. A pro spuštění nodu budete upravovat launch file `solution.launch.py`. 

### Doporučený postup
1) Přidejte připravený prázdný node Planning do `CMakeLists.txt`, zkompilujte jej a následně spusťe, např. pomocí launch filu.

> [!TIP]
> Budete potřebovat balík `nav_msgs`, lokalizujte jej pomocí: `find_package(nav_msgs REQUIRED)`.
>
> Dále bude nutné modifikovat řádky:  
> `set (dependencies`  
> `...`  
> `)`
>
> a
>
> `add_library(${PROJECT_NAME} SHARED`  
> `...`  
> `)`
>
> Přidejte také řádky:  
> `add_executable(<node_name> src/<file_name>.cpp)`  
> `target_link_libraries(<node_name> ${PROJECT_NAME})`  
> `install(TARGETS <node_name> DESTINATION lib/${PROJECT_NAME})`

2) Vytvořte klient pro načtení mapy prostředí a ověřte jeho funkčnost.

> [!TIP]
> Pro asynchronní odeslání požadavku na map server můžete využít následující řádek kódu:
>
> `auto future = <client>->async_send_request(<request>, bind(&<callback>, this, placeholders::_1));`
>
> Další informace k napsání klientu najdete v [tutoriálu](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html).

3) Implementujte obsluhu služby `plan_path` typu `nav_msgs/srv/GetPlan` a vyzkoušejte její volání z CLI.

> [!TIP]
> Návod k implementaci najdete v [tutoriálu](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html).
>
> Pro zavolání služby z CLI použijte příkaz:
>
> `ros2 service call <service_name> nav_msgs/srv/GetPlan "{start: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, goal: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, tolerance: 0.0}"`

4) Implementujte funkci `aStar()` pro naplánování trasy mezi body zaslanými v requestu služby. (Tip: parametr `tolerance` nemusíte nijak využít.) Výsledek publikujte do topicu `planned_path`.

> [!TIP]
> Při rekonstrukci trasy (po nalezení cílového uzlu) postupujete od cíle k počátku. Nezapomeňte proto nakonec sekvenci waypointů otočit, např. pomocí funkce `std::reverse`.
>
> Vyvarujte se vytváření více instancí struktury Cell se stejnými souřadnicemi!
>
> Z dokumentace zjistěte, zda je mapa uspořádáná jako row-major nebo column-major. Pro prevenci schizofrenie doporučuji použít stejné řazení i u pomocných proměnných (např. `closedList`). Připomeňte si, jak pomocí dvojice indexů (x, y) najít správný prvek v 1D poli.
>
> Nedoporučuji používat příliš komplikované datové typy z knihoven, kterým moc nerozumíte. Dobrou službu vám udělá i `std::vector`, jehož velikost nemusíte znát v době kompilace (narozdíl od obyčejného pole).

5) Upravte nastavení RVizu tak, aby zobrazoval trasu (display type Path). Konfiguraci uložte.
6) Ověřte funkčnost vašeho plánovacího algoritmu pro různé kombinace startovní a cílové pozice.
7) Implementujte funkci `dilateMap()`, která upraví podobu mapy použité pro plánování trasy. Zvolte vhodnou velikost odstupu od překážek. Zavolejte funkci ve vhodném místě v kódu.

> [!TIP]
> Implementace může být bez výčitek založena na vnořených `for` cyklech.

8) Implementujte funkci `smoothPath()`, která upraví trasu vygenerovanou algoritmem A* tak, aby byla hladká. Zvolte vhodné parametry použitého algoritmu.

> [!TIP]
> Doporučuji zastropovat počet iterací algoritmu, abyste se nedostali do nekonečné smyčky.

9) Ověřte funkčnost řešení jako celku.