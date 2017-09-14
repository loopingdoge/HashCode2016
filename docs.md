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

#### 1.2.1 L' Input e  l'output

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

| formula                            | significato                              |
| ---------------------------------- | ---------------------------------------- |
| at( drone0, coord( 1, 2))          | il drone con id drone0 è posizionato nella cella con riga 1 e colonna 2 |
| at( item1, drone0)                 | il drone con id drone0 sta trasportando un oggetto con id item1 |
| weighs( drone0, 30)                | il carico del drone con id drone0 è 30   |
| at( item0, warehouse0)             | l'oggetto con id item0 si trova al deposito con id warehouse0 |
| at( item3, order0)                 | l'oggetto con id item3 si trova dal cliente dell'ordine con id order0 |
| need( product0, order0)            | l'ordine con id order0 necessita di un prodotto con id product0 |
| delivering( item2, order0, drone1) | il drone con id drone0 sta ha preso in carico l'oggetto con item2 per consegnarlo all'ordine con id order0 |



Es. di un semplice stato del mondo

```prolog
at( drone0, coord( 0,0)) ∧ weighs( drone0, 0) ∧ at( item0, warehouse0) ∧ need( product0, order0)
```



Lo stato iniziale corrisponderà alla congiunzione delle formule atomiche rappresentanti i droni posizionati nella cella in alto a sinistra della mappa (coordinate 0,0), i droni scarichi, gli oggetti posizionati nei depositi e le richieste (necessità) dei clienti.

Lo stato finale o di goal corrisponderà alla congiunzione delle formule che vedono i vari oggetti (quelli che fanno parte di ordini) posizionati nelle abitazioni dei clienti. Questa corrisponde, ovviamente, ad una descrizione parziale di uno stato in cui il goal è soddisfatto.

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

- TODO pianificatore forward DFS

  ​










### 2.2 L'implementazione

#### 2.2.1 La scelta degli strumenti

Per la costruzione della soluzione si è scelto di implementare una nostra versione del noto pianificatore automatico STRIPS in Prolog.

Come interprete prolog si è scelta una particolare implementazione chiamata SWI-Prolog. 
SWI-Prolog presenta una storia trentennale, e risulta essere l'implementazione più utlizzata nelle università. La sua sintassi corrisponde completamente a quella originale. Il linguaggio estende Prolog aggiungendo alcune caratteristiche utili tra le quali la rappresentazione del testo in formato Unicode per facilitare lo scambio di informazioni con altri linguaggi e quindi paradigmi di programmazione.

- TODO, parlare della parte Python?



#### 2.3.1 Implementazione di STRIPS

- TODO scrivere qualcosa per introdurre

  ​

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

Fin quando questo non è verificato, si va a selezionare un operatore tramite il predicato *move* presente alla riga 8, che cercherà di fare match in maniera non deterministica con una clausola tra la *load* e la *deliver*, andando a recuperare le precondizioni, gli aggiungendi e i cancellandi.

Il predicato *condition_met*, presente alla riga 9, verifica se le precondizioni sono verificate nello stato corrente eseguendo l'unificazione delle variabili presenti.

Il predicato *change_state*, presente alla riga 10, esegue prima la differenza tra lo stato e i cancellandi, e poi inserisce gli aggiungendi creando di fatto il nuovo stato.

A questo punto di va a verificare che il nuovo stato non sia già stato visitato lungo il piano parziale tramite il predicato *not* e *member_state* presente alla riga 11. Se questo è verificato si aggiunge lo stato ad una pila tramte il predicato *stack* (riga 12) e si aggiunge l'operazione alla pila contenente il piano (riga 13).

Alla riga 14 è presente la chiamata ricorsiva con il nuovo stato e lo stesso goal.







------

### Bibliografia

SWI-Prolog,  http://www.swi-prolog.org/. URL consultato il 12 Settembre 2017
