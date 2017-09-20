# Delivery, Google HashCode 2016



#### Progetto del corso di Intelligenza artificiale AA 2016/2017

##### Corso di Laurea Magistrale in Informatica, Università di Bologna

##### Componenti del gruppo (in ordine alfabetico)

- Michel Bellomo  (matricola )

- Enrico Ceccolini  (matricola )

- Devid Farinelli  (matricola )

- Alberto Nicoletti  (matricola )

  ​

------



## 1. Il problema

Il problema affrontato in questo progetto porta il nome di **Delivery** e corrisponde al quesito posto da Google per l'HashCode 2016. Gli Hash Code sono competizioni dedicate alla programmazione che sfidano team di sviluppatori presenti in Europa, Medio Oriente e Africa a risolvere un problema concreto tra quelli che si trova ad affrontare l'azienda più influente del mondo informatico.

La traccia completa si trova [qui](https://hashcode.withgoogle.com/2016/tasks/hashcode2016_qualification_task.pdf/).

### 1.1 La consegna in breve

Internet ha cambiato in modo profondo il modo in cui facciamo acquisti ma non si è ancora arrivati alla fine del mutamento. Dopo aver effettuato l'acquisto occorre attendere diversi giorni prima di poter ricevere i prodotti nelle nostre abitazioni.

E' qui che trovano spazio i droni, veicoli elettrici che in autonomia possono spostarsi in volo evitando il traffico e di fatto riducendo le tempistiche per gli spostamenti di merci. Allora, data una flotta di droni, una lista di ordini e di disponibilità nei depositi come si possono pianificare le azioni da comunicare ai veicoli per completare le consegne nel minor tempo possibile?

### 1.2 Le specifiche 

La simulazione trova luogo in una griglia bidimensionale sulla quale i droni possono "volare". Ogni cella viene identificata dalla coordinata riga/colonna.

Sulla griglia trovano luogo i depositi (ognuno con coordinate differenti) e i destinatari degli ordini.

Ogni prodotto ha un tipo, il tipo specifica il peso.

I depositi contengono quantità definite di prodotti di diversi tipi, non per forza un deposito contiene prodotti di tutti i tipi. Durante la simulazione i depositi non vengono riforniti.

Gli ordini contengono le coordinate del cliente e specificano i prodotti richiesti. I prodotti richiesti possono avere tipi diversi e contenere più prodotti con lo stesso tipo.

I droni sono in grado di caricare un certo peso uguale per tutti e possono trasportare gli articoli dai depositi ai clienti. Per i loro spostamenti percorrono sempre la strada più breve (distanza Euclidea).

I droni sono in grado di eseguire i seguenti comandi:

- **Load:**  Vola fino alla coordinata del deposito specificato e carica un certo numero di prodotti di un certo tipo.
- **Deliver:** Vola fino alla coordinata del cliente e scarica un certo numero di prodotti di un certo tipo.

#### 1.2.1 L'Input e  l'output

Gli elementi ricevuti in input sono:

- la dimensione della griglia,
- il numero di droni disponibili,
- il numero massimo di turni (spiegati in seguito),
- il peso massimo trasportabile da un drone,
- i diversi tipi di prodotti e il corrispettivo peso,
- i depositi (coordinate e contenuto),
- gli ordini (coordinate di consegna e prodotti che ne fanno parte).

Gli elementi da fornire in output:

- la lista delle azioni che i droni devono eseguire (lista di Load e Deliver)

Ogni comando della soluzione consuma *d*+1 turni , dove *d* è la distanza percorsa dal drone per poter soddisfare la richiesta (0 se il drone si trova già in posizione).

#### 1.2.2 La valutazione della soluzione

La valutazione della soluzione trovata è definita da un punteggio che diminuisce all'aumentare del numero di turni in cui ogni ordine viene completato. Il punteggio finale consiste nella somma dei punti ottenuti per ogni ordine che viene completato entro il limite di turni (massimo numero di turni).

Risulta chiaro che soluzioni con un punteggio migliore sullo stesso input sono vantaggiose in termini di tempo di consegna degli ordini.



--------



## 2. La soluzione

Non partecipando effettivamente al contest, non siamo stati sottoposti agli stretti vincoli temporali per sottoporre la nostra soluzione. Per questo abbiamo deciso di rendere il problema più realistico, non considerando la clausola che vedeva non necessario il completamento di tutti gli ordini dati in input, soddisfacendo tutti i clienti.

Il nostro interesse si è spostato sulla ricerca dei comandi da fornire ai droni in modo da completare tutte le consegne in meno turni possibili invece che completare il maggior numero di ordini entro un numero definito di turni.

### 2.1 La progettazione

#### 2.1.1 La scelta dell'approccio

Per la risoluzione del problema, si è selezionato un approccio misto che vede alla base un sistema di **pianificazione automatica**, tecnica largamente utilizzata in ambito di trasporti.

Costruire un "piano" o una sequenza di azioni da poter fornire ai diversi droni per portare a termine le consegne è quello che ci occorre e la pianificazione come ricerca "ragiona" e agisce prorpio in questi termini.

Risulta immediata la definizione dello spazio degli stati che vedrà ogni stato rappresentato dalla posizione dei prodotti e dei droni sulla mappa. Come conseguenza si avrà una rappresentazione significativa dell'albero di ricerca che vedrà ogni nodo rappresentare uno stato e ogni arco un'azione compiuta da un drone. Definendo lo stato iniziale del mondo come la presenza dei prodotti nei depositi, e lo stato finale come la presenza dei prodotti ordinati nelle abitazioni dei clienti, si ha che la relazione tra la sequenza di operazioni che se eseguite a partire dallo stato iniziale provocano il raggiungimento di uno stato desiderato e le azioni che devono eseguire i vari droni, è immediata e non necessita di traduzione.

Come paradigma di programmazione, per la facilità e correttezza con cui potrà poi essere progettato il pianificatore, si è scelta la **programmazione logica**, in modo da poter rappresentare ed elaborare l'informazione tramite la logica del primo ordine.

Per affinare le capacità di ricerca dell'agente intelligente incaricato della ricerca nello spazio degli stati, l'approccio di pianificazione classico sarà esteso attraverso tecniche di constraint satisfaction.

#### 2.1.2 La conoscenza di base

Per rappresentare l'informazione viene sfruttata la logica del primo ordine. L'informazione ricevuta in input (descritta nella consegna) può essere codificata in modo da entrare a far parte della "conoscenza" del sistema formale.

Le informazioni che risultano valide prima, dopo e durante l'esecuzione dell'agente intelligente sono codificate come assiomi propri, esprimendo verità assoluta.

Tabella con esempi di assiomi propri (uno per ogni assioma proprio contemplato)

| assioma                                  | significato                              |
| ---------------------------------------- | ---------------------------------------- |
| drone(drone0).                           | drone0 è un drone (vengono creati *n* predicati per definire *n* droni distinti con *n* numero di droni in input) |
| payload(250).                            | il peso massimo trasportabile da un drone è 250 |
| product(product0, 35).                   | product0 è un tipo di prodotto e il peso del prodotto è 35 (per ogni tipo di prodotto è definito un predicato simile) |
| item(item0, product0).                   | item0 è un oggetto e il suo tipo di prodotto è product0 (ogni deposito contiene un numero precisato di istanze dello stesso prodotto, per ogni istanza è definito un predicato simile) |
| order(order0, [product2], coord(14, 27)). | order0 è un ordine, per essere soddisfatto necessita la consegna di un product2 e le coordinate del cliente sono x:14 y:27 |
| warehouse(warehouse0, coord(39, 15)).    | warehouse0 è un deposito e le sue coordinate sono x:39 y:15 |
| coord(X, Y) :- X\<50, X>=0, Y\<50, Y>=0. | la dimensione della griglia è 50x50      |



#### 2.1.3 Definizione dello spazio degli stati

Nella rappresentazione di uno stato influiscono i seguenti elementi:

- la posizione di ogni drone sulla mappa,
- il carico corrente di ogni drone,
- il peso corrente caricato di ogni drone,
- la posizione di ogni prodotto sulla mappa,
- la richiesta di un prodotto da parte di un cliente,
- la presa in carico della consegna di uno specifico prodotto ad uno specifico cliente da parte di un drone.

Tali elementi troveranno corrispondenza con formule atomiche definite tramite il paradigma logico, e lo stato sarà rappresentato dalla loro congiunzione.

Tabella con esempi di formule atomiche (uno per ogni formula contemplata)

| formula                           | significato                              |
| --------------------------------- | ---------------------------------------- |
| at(drone0, coord(1, 2))           | il drone con id drone0 è posizionato nella cella con riga 1 e colonna 2 |
| at(item1, drone0)                 | il drone con id drone0 sta trasportando un oggetto con id item1 |
| weighs(drone0, 30)                | il carico del drone con id drone0 è 30   |
| at(item0, warehouse0)             | l'oggetto con id item0 si trova al deposito con id warehouse0 |
| at(item3, order0)                 | l'oggetto con id item3 si trova dal cliente dell'ordine con id order0 |
| need(product0, order0)            | l'ordine con id order0 necessita di un prodotto con id product0 |
| delivering(item2, order0, drone1) | il drone con id drone0 sta ha preso in carico l'oggetto con item2 per consegnarlo all'ordine con id order0 |



Es. di un semplice stato del mondo

```prolog
at(drone0, coord( 0,0)) ∧ weighs(drone0, 0) ∧ at(item0, warehouse0) ∧ need(product0, order0)
```



Lo **stato iniziale** corrisponderà alla congiunzione delle formule atomiche rappresentanti i droni posizionati nella cella in alto a sinistra della mappa (coordinate 0,0), i droni scarichi (senza oggetti caricati), gli oggetti posizionati nei depositi e le richieste (necessità) dei clienti.

Lo **stato finale**, o di goal, corrisponderà alla congiunzione delle formule che vedono i vari oggetti (quelli che fanno parte di ordini) posizionati nelle abitazioni dei clienti. Questa corrisponde, ovviamente, ad una descrizione parziale di uno stato in cui il goal è soddisfatto.

Le operazioni che definiscono gli archi tra uno stato e un altro sono la **load** e la **deliver**. Per prima la *load* corrisponde all'azione di caricare su un drone un prodotto da un deposito. Tale operazione deve ottenere successo se è presente almeno un ordine che deve e che può essere soddisfatto. Si cerca e seleziona un drone e un deposito tra i disponibili a soddisfare la richiesta, e si definisce la presa in carico della consegna dell'oggetto da parte del drone. 

| LOAD              |                                          |
| ----------------- | ---------------------------------------- |
| **precondizioni** | at(Item, Warehouse), need(NeedId, Product, Order) |
| **aggiunte**      | at(Item, Drone), weighs(Drone, NewWeight), delivering(NeedId, Item, Order, Drone), at(Drone, Warehouse) |
| **cancellazioni** | at(Item, Warehouse), weighs(Drone, CurrentWeight), need(NeedId, Product, Order), at(Drone, PrevDroneLocation) |



La *deliver* corrisponde all'azione di consegnare ad un cliente, un oggetto precedentemente caricato su un drone. Tale operazione deve ottenere successo se è presente almeno un oggetto caricato su un drone. Si cerca e seleziona un drone tra quelli che hanno un oggetto da consegnare, e si effettua la consegna.

| DELIVER           |                                          |
| ----------------- | ---------------------------------------- |
| **precondizioni** | at(Item, Drone), delivering(NeedId, Item, Order, Drone) |
| **aggiunte**      | at(NeedId, Product, Order), weighs(Drone, NewWeight), at(Drone, Order) |
| **cancellazioni** | at(Item, Drone), weighs(Drone, CurrentWeight), delivering(NeedId, Item, Order, Drone), at(Drone, PrevDroneLocation) |





#### 2.1.4 L'albero di ricerca

Il numero di operazioni, come visto in precedenza, è fortemente limitato (sono solo due, load e deliver). Questo ha un impatto positivo sul branching factor dell'albero di ricerca. Tuttavia, lasciare totalmente libera l'unificazione delle variabili, può rendere questo vantaggio non sufficiente alla realizzazione di un pianificatore in avanti che sia efficente. La presenza di un numero alto di tipi di prodotto, aumenta la possibilità di vedere affidato ad un drone la consegna di un prodotto rivolta ad un cliente che non l'ha richiesto, unico fattore che provoca il dover ritrattare operazioni già inserite nel piano (backtraking). E' tuttavia possibile inserire il vincolo che lo impedisce, rendendo di interesse anche l'utilizzo di un **pianificatore in avanti** per l'esplorazione dello spazio degli stati. 

Le unificazioni per le altre variabili possono essere considerate sempre valide poiché stanno a rappresentare esclusivamente delle scelte: la scelta di un drone piuttosto che un altro, la scelta di consegnare un oggetto piuttosto di caricarne un altro, ecc. Quest'ultime azioni saranno sempre possibili al drone,  e rappresentano quindi un passo corretto verso ad una delle soluzioni.

Quest'ultimo fatto porta l'interesse verso la **ricerca in profondità**, non è necessario espandere per livelli occupando memoria inutilmente, proseguire in profondità porterà ad una soluzione. La vera differenza tra la scelta di una operazione e un'altra va vista in termini di impatto nel punteggio finale del piano che si sta costruendo, e non in possibilità o meno di raggiungerne uno valido.

Per guidare l'agente intelligente lungo questa ricerca occorre adottare stategie di unificazione adeguate, che abbiano forte impatto sul punteggio e di conseguenza sul tempo di attesa dei clienti e il consumo delle batterie dei droni che dovranno percorrere meno strada possibile. Quelle proposte da noi e che andremo ad implementare sono:

- scelta del deposito dove recuperare il prodotto in base alla distanza con il cliente,
- scelta del drone disponibile più vicino al deposito,
- scelta di utilizzare il drone più vicino al cliente e il deposito più vicino a quel drone,
- la casualità nel decidere di effettuare consegne anche prima di aver caricato completamente il drone (di aver eseguito tutte le load possibili)
- TODO Altro?




### 2.2 L'implementazione

#### 2.2.1 La scelta degli strumenti

Per la costruzione della soluzione si è scelto di implementare una nostra versione del noto pianificatore automatico STRIPS in Prolog.

Come interprete Prolog si è scelta una particolare implementazione chiamata SWI-Prolog. 
SWI-Prolog presenta una storia trentennale, e risulta essere l'implementazione più utlizzata nelle università. La sua sintassi corrisponde completamente a quella originale. Il linguaggio estende Prolog aggiungendo alcune caratteristiche utili tra le quali la rappresentazione del testo in formato Unicode per facilitare lo scambio di informazioni con altri linguaggi e quindi paradigmi di programmazione.

- TODO, parlare della parte Python?



#### 2.2.2 Implementazione di STRIPS

L'implementazione del pianificatore automatico con SWI-Prolog è banale. In stile Prolog, si definisce un predicato ricorsivo (*plan*), con una regola per il caso base e una regola per il caso ricorsivo. 

Codice del pianificatore, [ Fig.1 ]

```prolog
 1:  plan(State, Goal, _, Moves, _) :-
 2:      subset(Goal, State),
 3:      open('out/{{filename}}.cmds', write, Stream),
 4:      export_moves(Moves, Stream),
 5:      close(Stream).
 6:
 7:  plan(State, Goal, Been_list, Moves, MaxTurns) :-
 8:      move(State, Name, Preconditions, Actions),
 9:      conditions_met(Preconditions, State),
10:      change_state(State, Actions, Child_state),
11:      not(member_state(Child_state, Been_list)),
12:      stack(Child_state, Been_list, New_been_list),
13:      stack(Name, Moves, New_moves),
14:      plan(Child_state, Goal, New_been_list, New_moves, MaxTurns).
```

La ricerca in profondità si fermerà quando tutti i goal saranno verificati nello stato corrente, questo si verifica quando il predicato di sottoinsieme presente alla riga 2 sarà vero (l'insieme "goal" è sottoinsieme dello stato corrente).

Fin quando questo non è verificato, si va a selezionare un operatore tramite il predicato *move* presente alla riga 8, che cercherà di fare match con una clausola tra la *load* e la *deliver*, andando a recuperare le precondizioni, gli aggiungendi e i cancellandi.

Il predicato *condition_met*, presente alla riga 9, verifica se le precondizioni sono verificate nello stato corrente eseguendo, in maniera non deterministica, l'unificazione delle variabili non ancora assegnate.

Il predicato *change_state*, presente alla riga 10, esegue prima la differenza tra lo stato e i cancellandi, e poi inserisce gli aggiungendi creando di fatto il nuovo stato.

A questo punto di va a verificare che il nuovo stato non sia già stato visitato lungo il piano parziale tramite il predicato *not* e *member_state* presente alla riga 11. Se questo è verificato si aggiunge lo stato ad una pila tramte il predicato *stack* (riga 12) e si aggiunge l'operazione alla pila contenente il piano (riga 13).

Alla riga 14 è presente la chiamata ricorsiva con il nuovo stato e lo stesso goal.



#### 2.2.3 Implementazione della ricerca

Come riportato al paragrafo 2.1.4, per "guidare" la ricerca della sequenza di azioni si interviene sull'unificazione delle variabili. A livello imlementativo questo si ottiene inserendo nel corpo della regola *move* (richiamata alla riga 8 Fig.1) dei predicati al fine di implementare le strategie individuate.

Al fine di studio, la **prima strategia** considerata è quella di non prevedere alcuna strategia, e quindi di lasciare libera la ricerca. Il planner in questione è presente nel file "backtrack_stupid_planner.pl", l'unico controllo che effettua è quello sul peso del carico del drone (non si permette al drone di caricare oggetti se non può fisicamente). <u>Questo vincolo sarà implementato anche da tutte le successive strategie</u>.

Codice delle azioni del pianificatore *backtrack_stupid_planner.pl*, [ Fig.2 ]

```prolog
 1:	 move( State,
 2:		load(Drone, Product, Warehouse),
 3:  	... precondizioni, effetti ...
 4:  ) :-
 5:     drone(Drone), 
 6:		warehouse(Warehouse, _), 
 7:		order(Order, _, _), 
 8:		item(Item, Product),
 9:  	drone_load(State, Drone, CurrentWeight),
10:   	payload(MaxWeight),
11:    	product(Product, ProductWeight),
12:    	CurrentWeight + ProductWeight #=< MaxWeight,
13:    	NewWeight is CurrentWeight + ProductWeight.
14:
15:	 move( State,
16:		deliver(Drone, Product, Order),
17:  	... precondizioni, effetti ...
18:  ) :-
19:		drone_load(State, Drone, CurrentWeight),
20:     product(Product, ProductWeight),
21:     NewWeight is CurrentWeight - ProductWeight.
```

Il predicato *drone_load*, presente alla riga 9, recupera il peso attualemente caricato sul drone recuperato alla riga 5. Alla riga 11 si recupera il peso del prodotto che si vuole caricare e alla riga 12 si verifica che il peso non sia eccessivo.

La **seconda strategia** corrisponde ad un miglioramento sostanziale della prima. Introduce il vincolo che impedisce la possibilità di affidare ad un drone consegne di prodotti a clienti che non l'hanno richiesto, limitando notevolmente lo spazio degli stati. L'implementazione della strategia è presente nel file "stupid_planner.pl". <u>Questo controllo sarà implementato da tutte le successive strategie</u>.

Codice delle azioni del pianificatore *stupid_planner.pl*, [ Fig.2 ]

```prolog
 1:	 move(
 2:  	State,
 3:  	load(Drone, Product, Warehouse),
 4:  	... precondizioni, effetti ...
 5:	 ) :-
 6:  	drone(Drone),
 7:  	warehouse(Warehouse, _),
 8:  	order(Order, ProductList, _),
 9:  	member(Product, ProductList),
10:   	product(Product, ProductWeight),
11:  	item(Item, Product),
12:		... vincoli peso ...
13:    	requested_product_and_order(State, Order, Product, NeedId).
```

Il predicato *member*, presente alla riga 9, verifica che il prodotto sia effettivamente stato richiesto dall'ordine selezionato sopra. Il predicato *requested_product_and_order*, presente alla riga 13 verifica che l'ordine selezionato necessiti ancora di prodotti di un certo tipo.

La **terza strategia** è la prima che introduce l'idea di fare percorrere ai droni meno strada possibile, questione alla base dei trasporti. L'implementazione della strategia è presente nel file "planner.pl".

Codice delle azioni del pianificatore *planner.pl*, [ Fig.3 ]

```prolog
 1:	 move(
 2:  	State,
 3:  	load(Drone, Product, Warehouse),
 4:  	... precondizioni, effetti ...
 5:	 ) :-
 6:  	requested_product_and_order(State, Order, Product, NeedId),
 7:  	nearest_warehouse_from_order(State, Order, Product, Warehouse, Item),
 8:   	nearest_drone_from_warehouse(State, Warehouse, , Drone, CurrentWeight, NewWeight, Distance),
 9:    	drone_location(State, Drone, PrevDroneLocation),
10:    	... vincoli peso ...
```

Il predicato *nearest_warehouse_from_order* unifica la variabile "Warehouse" con il deposito più vicino al cliente. Il predicato  *nearest_drone_from_warehouse* unifica la variabile "Drone" con il drone disponibile (con possibilità di caricare il prodotto), più vicino al deposito.

Per costruzione, il pianificatore non aggiunge il piano azioni di *deliver* fino a che ci sia la possibilità di effettuare una *load*. Per contrastare questa priorità, la **quarta strategia** introduce un elemento aleatorio alla scelta dell'azione potenziale da aggiungere al piano. L'implementazione della strategia è presente nel file "random_action_planner.pl".



TODO cosa fa drones_planner.pl ?
La **quinta strategia** ...



### 2.3 Test

Tramite uno script verranno creati diversi input sulla quale le diverse strategie saranno confrontate in termini di tempo e punteggio. Ogni input corrisponderà ad una sessione di prova diversa.

Le varie sessioni verranno effettuate su una mappa 50x50, dove saranno dislocati 5 depositi contenenti prodotti di 5 tipi diversi. Il numero di oggetti per ordine varia da 1 a 3, gli oggetti da consegnare variano tra il numero di ordini e il numero di ordini moltiplicato per 3.

Nella **prima sessione** di prove sarà presente un solo drone ad effettuare le consegne.

Tabella che mostra i risultati al variare del numero di ordini da gestire.

| strategia                | tempo 30 ord. | punteggio 30 ord. | tempo 120 ord | punteggio 120 ord. | tempo 480 ord | punteggio 480 ord. |
| ------------------------ | :-----------: | :---------------: | :-----------: | :----------------: | :-----------: | :----------------: |
| backtrack_stupid_planner |    8.606s     |       1951        |      27m      |        1563        |   n.d. >1h    |        n.d.        |
| stupid_planner           |    5.586s     |       1968        |    18m54s     |        1669        |   n.d. >1h    |        n.d.        |
| planner                  |    1.183s     |       2413        |      58s      |        7904        |    13m53s     |        3611        |
| random_action_planner    |    0.885s     |       2390        |    32.350s    |        3572        |     9m29s     |        3630        |
| stupid_drones_planner    |    5.593s     |       1968        |    59.785s    |        4423        |    n.d >1h    |        n.d         |
| drones_planner           |    1.152s     |       2579        |    42.991s    |        4423        |     12m7s     |        6598        |



Nella **seconda sessione** di prove saranno presenti 5 droni.

Tabella che mostra i risultati al variare del numero di ordini da gestire.

| strategia                | tempo 30 ord. | punteggio 30 ord. | tempo 120 ord | punteggio 120 ord. | tempo 480 ord | punteggio 480 ord. |
| ------------------------ | :-----------: | :---------------: | :-----------: | :----------------: | :-----------: | :----------------: |
| backtrack_stupid_planner |   n.d. >1h    |       n.d.        |     n.d.      |        n.d.        |     n.d.      |        n.d.        |
| stupid_planner           |    34.695s    |       2660        |   n.d. >1h    |        n.d.        |     n.d.      |        n.d.        |
| planner                  |    1.660s     |       2590        |    37.179s    |        6554        |     16m7s     |        6133        |
| random_action_planner    |    1.429s     |       2763        |    29.076s    |       10200        |     8m8s      |       16677        |
| stupid_drones_planner    |    9.970s     |       2699        |   n.d. >1h    |        n.d         |     n.d.      |        n.d.        |
| drones_planner           |    1.560s     |       2809        |    45.940s    |       10475        |    14m36s     |       15475        |



Nella **terza sessione** di prove saranno presenti 40 droni. 

| strategia             | tempo 30 ord. | punteggio 30 ord. | tempo 120 ord | punteggio 120 ord. | tempo 480 ord | punteggio 480 ord. |
| --------------------- | :-----------: | :---------------: | :-----------: | :----------------: | :-----------: | :----------------: |
| planner               |    1.588s     |       2863        |    34.543s    |        6650        |     12m1s     |       13492        |
| random_action_planner |    1.597s     |       2842        |    22.438s    |       11397        |     6m25s     |       27385        |
| drones_planner        |    1.644s     |       2884        |    34.773s    |       11701        |    11m35s     |       28839        |



Nella **quarta sessione** di prove saranno presenti 40 droni e saranno 20 le tipologie di prodotti.

| strategia             | tempo 30 ord. | punteggio 30 ord. | tempo 120 ord | punteggio 120 ord. | tempo 480 ord | punteggio 480 ord. |
| --------------------- | :-----------: | :---------------: | :-----------: | :----------------: | :-----------: | :----------------: |
| planner               |    2.567s     |       2867        |     3m13s     |        9063        |    41m53s     |       12602        |
| random_action_planner |    2.480s     |       2840        |     1m36s     |       10998        |    22m52s     |       26725        |
| drones_planner        |    2.732s     |       2868        |     2m56s     |       11710        |    35m35s     |       28004        |



- TODO note:


- backtrack_stupid_planner, stupid_planner e stupid_drones_planner risentono gravemente dell'aumento del numero di droni.
- drones_planner contiene algoritmi pesanti ma ottiene punteggi migliori.
- l'elemento randomico non è controllabile, tempi ridotti ma punteggi a volte migliori a volte peggiori





------

### Bibliografia

SWI-Prolog,  http://www.swi-prolog.org/. URL consultato il 12 Settembre 2017
