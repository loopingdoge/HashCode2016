# Delivery, Google HashCode
**Progetto del corso di Intelligenza artificiale AA 2016/2017**
**Corso di Laurea Magistrale in Informatica, Università di Bologna**
**Componenti del gruppo (in ordine alfabetico)**

- Alberto Nicoletti  (819697) - alberto.nicoletti@studio.unibo.it
- Devid Farinelli  (819683)  - devid.farinelli@studio.unibo.it
- Enrico Ceccolini  (800490) - enrico.ceccolini3@studio.unibo.it
- Michel Bellomo  (819463) - michel.bellomo@studio.unibo.it

----------
# 1. Il problema

Il problema affrontato in questo progetto porta il nome di **Delivery**, e corrisponde al quesito posto da Google per l'Hash Code del 2016. Gli Hash Code sono competizioni dedicate alla programmazione che sfidano team di sviluppatori presenti in Europa, Medio Oriente e Africa a risolvere un problema concreto tra quelli affrontati dall'azienda più influente del mondo informatico. La traccia completa è disponibile [qui](https://storage.googleapis.com/coding-competitions.appspot.com/HC/2016/hashcode2016_qualification_task.pdf).


## 1.1 La consegna in breve

Internet ha cambiato profondamente il modo in cui facciamo acquisti, ma non si è ancora arrivati alla fine del mutamento. Dopo aver effettuato l'acquisto, occorre attendere diversi giorni prima di poter ricevere i prodotti nelle nostre abitazioni.
E' qui che trovano spazio i droni, veicoli elettrici che in autonomia possono spostarsi in volo evitando il traffico e di fatto riducendo le tempistiche per gli spostamenti di merci. Data una flotta di droni, una lista di ordini e di disponibilità nei depositi, come si possono pianificare le azioni da far eseguire ai velivoli per completare le consegne nel modo più efficiente?


## 1.2 Le specifiche 

La simulazione ha luogo in una griglia bidimensionale sulla quale i droni possono muoversi. Ogni cella viene identificata dalle sue coordinate di riga e colonna. Sulla griglia sono posizionati i **depositi** (ognuno con coordinate differenti) e i **destinatari** degli ordini.
Ogni **prodotto** ha un **tipo**, il tipo specifica anche il **peso**.
I depositi contengono quantità definite di prodotti di diversi tipi, durante la simulazione i depositi non vengono riforniti e un deposito può non contenere prodotti di tutti i tipi.
Ad ogni **ordine** sono associate le coordinate del cliente e i prodotti richiesti, i prodotti richiesti possono essere di diversi tipi e ci possono essere più prodotti dello stesso tipo.
I droni sono in grado di caricare un certo **peso** (uguale per tutti) e trasportano gli articoli dai depositi ai clienti percorrendo sempre la strada più breve (distanza Euclidea).
I droni sono in grado di eseguire le seguenti azioni:

- **Load:**  vola fino alla coordinata del deposito specificato e carica un certo numero di prodotti di un certo tipo.
- **Deliver:** vola fino alla coordinata del cliente e scarica un certo numero di prodotti di un certo tipo.
- **Wait**: attende nella posizione in cui si trova per un dato numero di turni.


## 1.2.1 L'input e l'output

I dati ricevuti in input sono:

- la dimensione della griglia
- il numero di droni disponibili
- il numero massimo di turni (definiti in seguito)
- il peso massimo trasportabile da un drone
- i diversi tipi di prodotti e il corrispettivo peso
- i depositi (coordinate e contenuto)
- gli ordini (coordinate di consegna e prodotti da consegnare)

I dati da fornire in output sono:

- la lista delle azioni che i droni devono eseguire (un elenco di Load, Deliver e Wait)

Ogni comando della soluzione consuma *d*+1 **turni**, dove *d* è la distanza percorsa dal drone per poter soddisfare la richiesta (d=0 se il drone si trova già nella posizione in cui eseguire l’azione).


## 1.2.2 La valutazione della soluzione

La valutazione della soluzione trovata, è definita da un punteggio che diminuisce all'aumentare del numero di turni utilizzati per completare ogni ordine. Il **punteggio finale** consiste nella somma dei punti ottenuti per ogni ordine. Vengono considerati solo gli ordini completati entro il limite di turni (massimo numero di turni).
Risulta chiaro che, minore è il tempo di consegna complessivo degli ordini, maggiore sarà il punteggio della soluzione.

----------
# 2. La soluzione

Non partecipando effettivamente alla competizione, non siamo stati sottoposti agli stretti vincoli temporali che imponevano di sottoporre una soluzione entro 4 ore e per questo abbiamo deciso di rendere il problema più realistico non considerando la clausola che permetteva di non portare a termine tutti gli ordini dati in input. Abbiamo quindi lavorato a soluzioni che completassero tutti gli ordini, calcolando comunque i punteggi tenendo conto del limite di turni. Il nostro interesse si è focalizzato sul convergere nel minor tempo possibile verso una soluzione con un buon punteggio, piuttosto che sul completare il maggior numero di ordini entro un numero definito di turni. Rispettando il formato dell’output, produrremo tuttavia risultati compatibili.


## 2.1 La progettazione
## 2.1.1 La scelta dell'approccio

Per la risoluzione del problema, si è selezionato un approccio misto che vede alla base un sistema di **pianificazione automatica**, tecnica largamente utilizzata in ambito decisionale riguardante i trasporti. Costruire un *piano*, o una sequenza di azioni, da poter fornire ai diversi droni per portare a termine le consegne è quello che ci occorre e, la pianificazione come ricerca, "ragiona" e agisce proprio in questi termini.
Risulta immediata la definizione dello **spazio degli stati**, che vedrà ogni stato rappresentato dalle coordinate di ogni drone, lo stato di ogni ordine e la posizione di ciascun item.
La sequenza di operazioni che, se eseguite a partire dallo stato iniziale, provocano il raggiungimento di uno stato desiderato, corrisponderà perfettamente con le azioni che i droni devono eseguire.
Come paradigma di programmazione, per la facilità e correttezza con cui potrà poi essere progettato il pianificatore, si è scelta la **programmazione logica**, in modo da poter rappresentare ed elaborare l'informazione tramite la logica del primo ordine.
Per affinare le capacità di ricerca dell'agente intelligente incaricato della ricerca nello spazio degli stati, l'approccio di pianificazione classico sarà esteso attraverso tecniche di **constraint satisfaction**.


## 2.1.2 La conoscenza di base

Per rappresentare l'informazione viene sfruttata la logica del primo ordine. L'informazione ricevuta in input (descritta nella consegna), può essere codificata in modo da entrare a far parte della *conoscenza* del nostro sistema formale.
Le informazioni che risultano valide prima, dopo e durante l'esecuzione dell'agente intelligente sono codificate come assiomi propri, esprimendo verità assoluta.

Segue l’elenco degli assiomi propri contemplati:

| **assioma**                               | **significato**                                                                                                                                                                        |
| ----------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| drone(drone0).                            | drone0 è un drone (vengono creati *n* predicati per definire *n* droni distinti con *n* numero di droni in input)                                                                      |
| payload(250).                             | il peso massimo trasportabile da un drone è 250                                                                                                                                        |
| product(product0, 35).                    | product0 è un tipo di prodotto e il peso del prodotto è 35 (per ogni tipo di prodotto è definito un predicato simile)                                                                  |
| item(item0, product0).                    | item0 è un oggetto e il suo tipo di prodotto è product0 (ogni deposito contiene un numero precisato di istanze dello stesso prodotto, per ogni istanza è definito un predicato simile) |
| order(order0, [product2], coord(14, 27)). | order0 è un ordine, per essere soddisfatto necessita la consegna di un product2 e le coordinate del cliente sono x:14 y:27                                                             |
| warehouse(warehouse0, coord(39, 15)).     | warehouse0 è un deposito e le sue coordinate sono x:39 y:15                                                                                                                            |
| coord(X, Y) :- X<50, X>=0, Y<50, Y>=0.    | la dimensione della griglia è 50x50                                                                                                                                                    |



## 2.1.3 Definizione dello spazio degli stati

Nella rappresentazione di uno stato è comprende i seguenti dati:

- le coordinate di ogni drone sulla mappa
- il carico di ogni drone
- il peso del carico di ogni drone
- il contenuto di ogni warehouse
- lo stato di ciascun prodotto di ogni ordine, che può essere:
    - in attesa di essere caricato
    - in viaggio verso le destinazione
    - consegnato

Tali elementi corrispondono a formule atomiche definite tramite il paradigma logico, e lo stato è rappresentato dalla loro congiunzione. Seguono le formule atomiche utilizzate:

| **formula**                       | **significato**                                                                                        |
| --------------------------------- | ------------------------------------------------------------------------------------------------------ |
| at(drone0, coord(1, 2))           | il drone con id drone0 è posizionato nella cella con riga 1 e colonna 2                                |
| at(item1, drone0)                 | il drone con id drone0 sta trasportando un oggetto con id item1                                        |
| weighs(drone0, 30)                | il carico del drone con id drone0 è 30                                                                 |
| at(item0, warehouse0)             | l'oggetto con id item0 si trova al deposito con id warehouse0                                          |
| at(item3, order0)                 | l'oggetto con id item3 si trova dal cliente dell'ordine con id order0                                  |
| need(product0, order0)            | l'ordine con id order0 necessita di un prodotto con id product0                                        |
| delivering(item2, order0, drone1) | il drone con id drone0 ha preso in carico l'oggetto con item2 per consegnarlo all'ordine con id order0 |


Es. di un semplice stato del mondo

    at(drone0, coord(0,0)) ∧ weighs(drone0, 0) ∧ at(item0, warehouse0) ∧ need(product0, order0)

Lo **stato iniziale**, corrisponde alla congiunzione delle formule atomiche rappresentanti i droni senza carico posizionati nella cella in alto a sinistra della mappa (coordinate 0,0), gli oggetti posizionati nei depositi e le specifiche degli ordini dei clienti.
Lo **stato finale**, o di goal, corrisponde alla congiunzione delle formule che vedono i vari oggetti (quelli che fanno parte di ordini) posizionati nelle abitazioni dei clienti. Questa corrisponde ad una descrizione parziale di uno stato in cui il goal è soddisfatto.
Le **operazioni** sono la load, la *deliver* e la *wait*.

## L’operazione Load

Corrisponde all'azione di caricare su un drone un prodotto da un deposito. Tale operazione è permessa se è presente almeno un ordine che deve essere soddisfatto. Si cerca e seleziona un drone ed un deposito tra i disponibili a soddisfare la richiesta, e si definisce la presa in carico della consegna dell'oggetto da parte del drone. 

| **LOAD**          |                                                              |
| ----------------- | ------------------------------------------------------------ |
| **precondizioni** | at(Item, Warehouse), at(Drone, PrevDroneLocation), need(NeedId, Product, Order) |
| **aggiunte**      | at(Item, Drone), weighs(Drone, NewWeight), delivering(NeedId, Item, Order, Drone), at(Drone, Warehouse) |
| **cancellazioni** | at(Item, Warehouse), weighs(Drone, CurrentWeight), need(NeedId, Product, Order), at(Drone, PrevDroneLocation) |

## L’operazione Deliver

Corrisponde all'azione di consegnare ad un cliente un oggetto precedentemente caricato su un drone. Tale operazione è permessa se e solo se è presente almeno un oggetto caricato su un drone. Si cerca e seleziona un drone tra quelli che hanno un oggetto da consegnare, e si effettua la consegna.

| **DELIVER**       |                                                                                                                     |
| ----------------- | ------------------------------------------------------------------------------------------------------------------- |
| **precondizioni** | at(Item, Drone), at(Drone, PrevDroneLocation), delivering(NeedId, Item, Order, Drone)                               |
| **aggiunte**      | at(NeedId, Product, Order), weighs(Drone, NewWeight), at(Drone, Order)                                              |
| **cancellazioni** | at(Item, Drone), weighs(Drone, CurrentWeight), delivering(NeedId, Item, Order, Drone), at(Drone, PrevDroneLocation) |

## L’operazione Wait

Corrisponde all’azione di attesa di un drone nella posizione corrente per un certo numero di turni.
Tale operazione è sempre permessa.


## 2.1.4 L'albero di ricerca

Il numero di operatori è fortemente limitato (sono solo tre, load, deliver e wait), ma il numero di oggetti sulla quale possono essere eseguiti risulta, almeno per problemi realistici, molto elevato (es. flotta di droni numerosa, vasta scelta di prodotti, un grande numero di magazzini o per grandi quantità di ordini). Questo ci porta ad avere un branching factor notevole ma è anche vero che guidando l’unificazione è possibile evitare la possibilità di vedere affidato ad un drone la consegna di un prodotto rivolta ad un cliente che non l’ha richiesto e quindi di allontanarci da uno stato goal e mantenere l’interesse per una soluzione che preveda l’utilizzo di un pianificatore in avanti per l’esplorazione dello spazio degli stati.
Le unificazioni per le altre variabili, possono essere considerate sempre valide, poiché stanno a rappresentare esclusivamente delle scelte: la scelta di un drone piuttosto che un altro, la scelta di consegnare un oggetto caricato piuttosto di caricarne altri, ecc. Queste ultime azioni saranno sempre possibili al drone, e rappresentano quindi un passo corretto verso ad uno stato goal.
Quest'ultimo fatto porta l'interesse verso la **ricerca in profondità**, non è necessario espandere per livelli occupando memoria inutilmente, proseguire in profondità porterà ad una soluzione. La vera differenza tra la scelta di una operazione e un'altra va vista in termini di impatto nel punteggio finale del piano che si sta costruendo, e non in possibilità o meno di raggiungerne uno valido.
Per guidare l'agente intelligente lungo questa ricerca occorre adottare strategie di unificazione adeguate, che abbiano un impatto positivo sul punteggio e di conseguenza sul tempo di attesa dei clienti e il consumo delle batterie dei droni che dovranno percorrere meno strada possibile. Quelle individuate da noi e che andremo ad implementare sono:

- scelta del deposito dove recuperare il prodotto in base alla distanza con il cliente,
- scelta del drone disponibile più vicino al deposito
- la casualità nel decidere di effettuare consegne anche prima di aver caricato completamente il drone (di aver eseguito tutte le load possibili)
- scelta di utilizzare il drone più vicino al cliente e il deposito più vicino a quel drone
- distribuire equamente il lavoro associando una azione a ciascun drone ad ogni passo della pianificazione


## 2.2 L'implementazione
## 2.2.1 La scelta degli strumenti

Per la costruzione della soluzione si è scelto di implementare una nostra versione del noto pianificatore automatico **STRIPS** in Prolog.
Come interprete Prolog si è scelta una particolare implementazione chiamata **SWI-Prolog**. 
SWI-Prolog presenta una storia trentennale, e risulta essere l'implementazione più utilizzata in ambito universitario. La sua sintassi corrisponde in totalità a quella originale con l’aggiunta di alcune caratteristiche utili tra le quali la rappresentazione del testo in formato Unicode per facilitare lo scambio di informazioni con altri linguaggi e paradigmi di programmazione.
Data la necessità di automatizzare le fasi di esecuzione (parsing, planning, output e calcolo punteggio) si è preferito aggiungere una sovrastruttura parametrica che permettesse di specificare un planner e l’input sul quale eseguirlo e generasse in output un file di azioni e ne calcolasse lo score.
A tal scopo si è scelto **Python** per la sua facile gestione di file di testo strutturati e per la facilità di risolvere i problemi prima citati tramite appositi script. Abbiamo quindi creato i seguenti script:

- *execute.py*, permette di eseguire un determinato planner su una determinato file di input, generando l’output e calcolandone lo score
- *planner_comparison.py,* permette di eseguire una batteria di test con tutti i planner su un determinato input
- *parser.py,* legge un determinato input e genera il relativo file Prolog contenente la base di conoscenza, lo stato iniziale e il goal secondo la sintassi descritta nei paragrafi precedenti
- *cmds2out.py,* converte una lista di azioni, prodotte da un pianificatore Prolog in una sequenza di output secondo la sintassi descritta dalla traccia di Google
- *scoring.py,* legge il file di output di una determinata soluzione e ne calcola il punteggio
- *generate_input.py,* permette la generazione di file di input procedurali con caratteristiche personalizzate


## 2.2.2 Implementazione di STRIPS

L'implementazione del pianificatore automatico con SWI-Prolog risulta semplice, si definisce un predicato ricorsivo (*plan*) con una regola per il caso base e una regola per il caso ricorsivo: 

Codice delle pianificazione di *orders_***.pl*, [ Fig.1 ]

    plan(State, Goal, _, Moves, _) :-
      subset(Goal, State),
      open('out/{{filename}}.cmds', write, Stream),
      export_moves(Moves, Stream),
      close(Stream).
    
    plan(State, Goal, Been_list, Moves, MaxTurns) :-
      move(State, Name, Preconditions, Actions),
      conditions_met(Preconditions, State),
      change_state(State, Actions, Child_state),
      not(member_state(Child_state, Been_list)),
      stack(Child_state, Been_list, New_been_list),
      stack(Name, Moves, New_moves),
      plan(Child_state, Goal, New_been_list, New_moves, MaxTurns).

La ricerca in profondità si fermerà quando tutti i goal saranno verificati nello stato corrente, ossia quando il predicato di sottoinsieme (riga 2) sarà vero (l'insieme "goal" è sottoinsieme dello stato corrente).
Fin quando questo non risulta vero, si va a selezionare una azione tramite il predicato *move* (riga 8), che cercherà di scegliere match con una clausola tra la *load,* la *deliver* e la *wait*, ottenendo le precondizioni da verificare, le aggiunte e le cancellazioni.
Il predicato `conditions_met` (riga 9), verifica se le precondizioni dell’azione scelta sono verificate nello stato corrente eseguendo l'unificazione delle variabili non ancora assegnate.
Il predicato `change_state` (riga 10), aggiorna lo stato applicando le cancellazioni e le aggiunte.
Alla riga 14 è presente la chiamata ricorsiva con il nuovo stato e lo stesso goal.

## 2.2.3 Implementazione della ricerca

Come riportato al paragrafo 2.1.4, per "guidare" la ricerca della sequenza di azioni si interviene sull'unificazione delle variabili. A livello implementativo questo si ottiene inserendo nel corpo della regola `move` (richiamata alla riga 8 Fig.1) dei predicati al fine di implementare le strategie individuate.

Al fine di studio, la **prima strategia** considerata è quella di non prevedere alcuna strategia, e quindi di lasciare libera la ricerca. Il planner in questione è presente nel file *orders_dfs.pl*, l’unico controllo effettuato è necessario all’ammissibilità dell’azione scelta, ovvero il drone non può caricare un oggetto se la somma di `(peso_trasportato_dal_drone + peso_oggetto)` supera il peso massimo trasportabile dal drone.
Questo vincolo sarà implementato anche da tutte le successive strategie.

Codice delle azioni del pianificatore *orders_dfs.pl*, [ Fig.2 ]

    move(State,
      load(Drone, Product, Warehouse),
      ... precondizioni,
      ... effetti
    ) :-
        requested_product_and_order(State, Order, Product, NeedId),
        order(Order, ProductList, _),
        member(Product, ProductList),
        product(Product, _),
        warehouse(Warehouse, _),
        item(Item, Product),
        drone(Drone),
        drone_load(State, Drone, CurrentWeight),
        payload(MaxWeight),
        product(Product, ProductWeight),
        CurrentWeight + ProductWeight #=< MaxWeight,
        NewWeight is CurrentWeight + ProductWeight,
        distance(State, Drone, Warehouse, Distance),
        TurnsConsumed is Distance + 1.
    
    move(State,
      deliver(Drone, Product, Order),
      ... precondizioni, effetti ...
    ) :-
        delivering_product_and_order(State, Order, Item, NeedId, Drone),
        order(Order, ProductList, _),
        member(Product, ProductList),
        item(Item, Product),
        drone(Drone),
        drone_load(State, Drone, CurrentWeight),
        product(Product, ProductWeight),
        NewWeight is CurrentWeight - ProductWeight,
        distance(State, Drone, Order, Distance),
        TurnsConsumed is Distance + 1.

Il predicato `drone_load` (riga 9), recupera il peso attualmente caricato sul drone (drone recuperato alla riga 5). Alla riga 11 si recupera il peso del prodotto che si vuole caricare, e alla riga 12 si verifica che il peso non sia eccessivo.
Si notino i predicati `requested_product_and_order` (riga 6) e `delivering_product_and_order` (riga 25): questi sono un’ottimizzazione realizzata tramite una ricerca nello `Stato` per guidare l’unificazione, rispettivamente, delle coppie di variabili `(order, product)` e `(order, item)` verso *ordini* ancora da gestire che contengono un determinato *prodotto* o *oggetto*. Questa ottimizzazione introduce un miglioramento sostanziale nel tempo di calcolo.

La **seconda strategia** (*orders_shortest.pl*) introduce l'idea di fare percorrere ai droni meno strada possibile, questione alla base dei trasporti. Durante l’operazione di *load* vengono quindi introdotti i predicati `nearest_warehouse_from_order` e `nearest_drone_from_warehouse`.
Il primo unifica la variabile `Warehouse` con il deposito più vicino al cliente. Il secondo unifica la variabile `Drone` con il drone disponibile (con possibilità di caricare il prodotto) più vicino al deposito. Questo corrisponde a scegliere prima un *magazzino* e poi un *drone*, prendendo quelli che minimizzano la distanza. La scelta dell’ordine da gestire rimane invece invariata.

Codice delle azioni del pianificatore orders_shortest*.pl*, [ Fig.3 ]

    move(
      State,
      load(Drone, Product, Warehouse),
      ... precondizioni, effetti ...
    ) :-
        requested_product_and_order(State, Order, Product, NeedId),
        order(Order, ProductList, _),
        member(Product, ProductList),
        product(Product, _),
        nearest_warehouse_from_order(State, Order, Product, Warehouse, Item),
        nearest_drone_from_warehouse(State, Warehouse, Product, Drone, CurrentWeight, NewWeight, Distance),
        warehouse(Warehouse, _),
        item(Item, Product),
        drone(Drone),
        TurnsConsumed is Distance + 1.

La **terza strategia** (*orders_random.pl*) introduce un elemento aleatorio alla scelta dell'azione potenziale da aggiungere al piano. Nelle soluzioni precedenti, il pianificatore non aggiunge mai al piano di azioni delle *deliver* fino a che c’è la possibilità di effettuare una *load,* questo pianificatore invece prova ad aggiungere una *deliver* o una *load* con la stessa probabilità.

Codice della scelta casuale dell’azione da scegliere nel pianificatore *orders_shortest_random,* [Fig. 4]

    plan(State, Goal, _, Moves, MaxTurns) :-
        % number of items loaded by the drones at the moment
        nb_getval(counterLoad, CounterValue),
        MovesNames = [load, deliver],
        random_member(Selection, MovesNames),
    
        % if
        (
          Selection == deliver % if random move is "deliver"
            ->
                (
                   CounterValue == 0 % if there is nothing to deliver exec a load
                      ->
                        move(load, State, Name, Preconditions, Actions)
                      ;
                        move(deliver, State, Name, Preconditions, Actions)
                )
            ;
            % else try load, if it fail use deliver
            move(_, State, Name, Preconditions, Actions)
        ),
    
        conditions_met(Preconditions, State),
        write(Name), nl,
        change_state(State, Actions, Child_state),
        not(member_state(Child_state, Been_list)),
        stack(Child_state, Been_list, New_been_list),
        stack(Name, Moves, New_moves),
        plan(Child_state, Goal, _, New_moves, MaxTurns).

Un problema riscontrato nei pianificatori discussi finora è che non viene impiegata nessuna strategia per distribuire equamente il lavoro tra i droni disponibili.
Vengono quindi introdotti i seguenti pianificatori:

- *drones_dfs*
- *drones_shortest*
- *drones_shortest_random*

Questi impiegano le stesse strategie descritte sopra per i pianificatori `orders_*` aventi il medesimo suffisso (*dfs/shortest/shortest_random*), ma differiscono nell’implementazione del predicato `plan`. Questo viene modificato per considerare ogni *drone* disponibile ed assegnargli obbligatoriamente un’azione, eventualmente anche di *wait*.

Codice del predicato `plan` nei pianificatori *drones_*,* [Fig. 5]

    plan(State, Goal, BeenList, Moves, MaxTurns) :-
        bagof(Drone, drone(Drone), DroneList),
        drones_move(DroneList, State, BeenList, Moves, ChildState, NewBeenList, NewMoves),
        not(member_state(ChildState, BeenList)),
        plan(ChildState, Goal, NewBeenList, NewMoves, MaxTurns).
    
    drones_move([], State, BeenList, Moves, OutState, OutBeenList, OutMoves) :-
        OutState = State,
        OutBeenList = BeenList,
        OutMoves = Moves.
    drones_move([Drone|Tail], State, BeenList, Moves, OutState, OutBeenList, OutMoves) :-
        move(Drone, State, Name, Preconditions, Actions),
        conditions_met(Preconditions, State),
        write(Name), nl,
        change_state(State, Actions, ChildState),
        stack(ChildState, BeenList, NewBeenList),
        stack(Name, Moves, NewMoves),
        drones_move(Tail, ChildState, NewBeenList, NewMoves, OutState, OutBeenList, OutMoves).

Tramite il predicato `bagof` (riga 2) vengono collezionati tutti i droni disponibili, e tramite la chiamata `drones_move` viene trovata sequenzialmente un’azione per ognuno di loro.

## 2.3 Test

Tramite uno script sono stati creati diversi input sulla quale le diverse strategie sono confrontate in termini di tempo e punteggio. Ogni input corrisponde ad una sessione di prova diversa.
Le varie sessioni sono effettuate su una mappa 50x50, dove sono dislocati 5 depositi. Il numero di oggetti per ordine varia da 1 a 3, quindi gli oggetti da consegnare variano tra il numero di ordini e il numero di ordini moltiplicato per 3.
Nel calcolo del punteggio non viene tenuto conto del numero massimo di turni. Nel caso si imponesse un numero massimo di turni stringente, il punteggio ne risentirebbe.

Per le prime tre sessioni le tipologie di prodotti sono 5.
Nella **prima sessione** di prove è presente un solo drone per effettuare le consegne.

Tabella dei risultati al variare del numero di ordini da gestire.

| **Strategia**              | **30 ordini**<br>tempo / punteggio / turni | **120 ordini**<br>tempo / punteggio / turni | **480 ordini**<br>tempo / punteggio / turni |
| -------------------------- | ------------------------------------------ | ------------------------------------------- | ------------------------------------------- |
| **orders_dfs**             | 0.095 / 2708 / 2633                        | 2.830 / 8447 / 12090                        | 120.158 / 27010 / 46066                     |
| **orders_shortest**        | 0.856 / 2759 / 1955                        | 104.996 / 9797 / 7327                       | n.d.                                        |
| **orders_shortest_random** | 0.870 / 2768 / 1937                        | 78.896 / 9797 / 7327                        | 3014.436 / 30308 / 38041                    |
| **drones_dfs**             | 0.060 / 2708 / 2633                        | 1.571 / 8447 / 12090                        | 95.301 / 27010 / 46066                      |
| **drones_shortest**        | 1.170 / 2802 / 1571                        | 97.218 / 10293 / 5966                       | n.d                                         |
| **drones_shortest_random** | 0.655 / 2801 / 1604                        | 70.504 / 10293 / 5966                       | 2507.569 / 30483 / 35735                    |


Nella **seconda sessione** di prove sono presenti 5 droni.

Tabella dei risultati al variare del numero di ordini da gestire.

| **strategia**              | **30 ordini**<br>tempo / punteggio / turni | **120 ordini**<br>tempo / punteggio / turni | **480 ordini**<br>tempo / punteggio / turni |
| -------------------------- | ------------------------------------------ | ------------------------------------------- | ------------------------------------------- |
| **orders_dfs**             | 0.549 / 2865 / 1968                        | 21.230 / 9788 / 8030                        | 702.253 / 36408 / 25953                     |
| **orders_shortest**        | 2.361 / 2877 / 1285                        | 112.608 / 10358 / 6128                      | n.d.                                        |
| **orders_shortest_random** | 0.840 / 2899 / 531                         | 56.574 / 11404 / 2198                       | 2291.359 / 45592 / 4807                     |
| **drones_dfs**             | 0.129 / 2896 / 671                         | 3.488 / 11112 / 2528                        | 112.429 / 43822 / 8726                      |
| **drones_shortest**        | 2.183 / 2900 / 358                         | 118.081 / 11526 / 1570                      | n.d.                                        |
| **drones_shortest_random** | 1.415 / 2900 / 352                         | 48.630 / 11590 / 1565                       | 2640.090 / 46175 / 4807                     |


Nella **terza sessione** di prove sono presenti 40 droni. 

Tabella dei risultati al variare del numero di ordini da gestire.

| **strategia**              | **30 ordini**<br>tempo / punteggio / turni | **120 ordini**<br>tempo / punteggio / turni | **480 ordini**<br>tempo / punteggio / turni |
| -------------------------- | ------------------------------------------ | ------------------------------------------- | ------------------------------------------- |
| **orders_dfs**             | 2.770 / 2894 / 564                         | 114.228 / 10857 / 5736                      | n.d.                                        |
| **orders_shortest**        | 0.947 / 2897 / 383                         | 39.499 / 11338 / 3176                       | 3312.960 / 36595 / 28036                    |
| **orders_shortest_random** | 0.752 / 2887 / 361                         | 21.118 / 11841 / 495                        | 1466.043 / 47506 / 2060                     |
| **drones_dfs**             | 0.347 / 2900 / 163                         | 5.298 / 11838 / 385                         | 165.246 / 47549 / 1325                      |
| **drones_shortest**        | 1.358 / 2900 / 124                         | 43.921 / 11894 / 242                        | n.d.                                        |
| **drones_shortest_random** | 1.157 / 2900 / 127                         | 30.387 / 11889 / 288                        | 2614.957 / 47699 / 1421                     |


Si osserva che:

- Un pianificatore dalle euristiche più complesse aumenta effettivamente il punteggio massimo. Questo dato emerge poco, in quanto nei test si è scelto un numero massimo di turni che fosse soddisfatto da ogni pianificatore. Il dato da tenere in considerazione è quindi il rapporto (*punteggio/turni*);
- All’aumentare dei controlli eseguiti dal pianificatore per ottimizzare globalmente il carico di lavoro di ciascun drone, aumenta di molto il tempo di esecuzione. Un esempio è *orders_shortest* che ottimizza il percorso dei droni portando ad un punteggio migliore ma richiedendo molto più tempo di calcolo;
- Il fattore “random” aggiunto ai *planner*, ha in generale effetti molto positivi per quanto riguarda i tempi (circa dimezzati), ottenendo punteggi simili ai pianificatori *shortest*;
- Come ci si aspettava, i pianificatori *drones* risultano sempre migliori delle controparti *orders*.

Un’ulteriore osservazione riguarda il rapporto tra “bontà” della pianificazione e i tempi di calcolo: si nota che all’aumentare del numero di ordini, iniziano a comparire nelle tabelle dei valori n.d. Questi indicano la morte del processo dopo un’ora di calcolo. Questa scelta, oltre a essere fatta per questioni di mero tempo, ci permette anche di dare un’informazione importante sulla possibilità di usare la nostra soluzione in un contesto reale.

# 3. Conclusioni

In questo documento abbiamo riassunto il nostro lavoro di progetto, cosa ci ha spinto a svilupparlo in questa direzione, e come abbiamo cercato di valutare e validare la nostra soluzione in modo concreto, avvalendoci anche degli strumenti forniti durante la gara di partenza.
Ci sono, tuttavia, delle considerazioni finali da fare sull’intero progetto.
Si noti che non sono riportati test eseguiti sui piani di consegna forniti da Google con la traccia della sfida: in una prima fase di test del codice abbiamo provato ad eseguirli, e ci abbiamo riprovato dopo una serie di ottimizzazioni ma queste non terminano a causa dell’esaurimento della memoria disponibile. Da ciò si evince che, se da un lato un ambiente come il Prolog risulta particolarmente naturale per la descrizione di un problema del genere, specialmente tramite l’implementazione di un pianificatore, dall’altro non è il migliore degli approcci dal punto di vista pratico su casi d’uso reali. Va aggiunto inoltre, che il problema non riguarda solo l’uso di memoria ma anche dal punto di vista del tempo di esecuzione: l’esaurimento della memoria si verifica solo dopo svariati minuti di calcolo (su un computer casalingo). Questo probabilmente accade a causa dell’enorme fattore di branching dell’albero di ricerca: sebbene si possano eseguire solo due azioni (ovvero load e delivery), è possibile compierle da più agenti su un numero non banale di oggetti. Questo indica che se anche si riuscisse a ottimizzare l’uso della memoria, o anche avendo a disposizione delle risorse esagerate, sarebbe incredibilmente complesso agire sul fattore tempo.
Risulta quindi chiaro il motivo dei turni limitati nella sfida originale: ipotizziamo che l’idea fosse quella di spingere verso delle soluzioni greedy che sacrifichino un approccio che esplori l’albero degli stati ma che si provi a ragionare in modo differente, come un paradigma distribuito.
Alla luce di questa esperienza, come gruppo, crediamo che questo approccio non sia dei migliori: per affrontare questo tipo di sfida, cambieremmo completamente approccio, passando da una pianificazione classica dove l’agente principale è il pianificatore stesso, ad un ambiente multi-agente, magari nativamente distribuito in cui il focus della pianificazione sia invece spostata sui droni. Questo pone degli interrogativi sui limiti delle tecnologie dell’Intelligenza artificiale classica e su come si possa fare per adeguarla ai nuovi problemi a cui sono richieste delle soluzioni intelligenti.

----------
# Bibliografia

[1] SWI-Prolog, http://www.swi-prolog.org. URL consultato il 12 Settembre 2017
[2] Stuart J. Russel e Peter Norving, Artificial Intelligence, Pearson, 2017, ISBN978-93-325-4351-5.


