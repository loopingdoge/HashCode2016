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



## Il problema

Il problema affrontato in questo progetto porta il nome di **Delivery** e corrisponde al quesito posto da Google per l'HashCode 2016. Gli Hash Code sono competizioni dedicate alla programmazione che sfidano team di sviluppatori presenti in Europa, Medio Oriente e Africa a risolvere un problema concreto tra quelli che si trova ad affrontare l'azienda più influente del mondo informatico.

La traccia completa si trova [qui](https://hashcode.withgoogle.com/2016/tasks/hashcode2016_qualification_task.pdf/).

### La consegna in breve

Internet ha cambiato in modo profondo il modo in cui facciamo acquisti ma non si è ancora arrivati alla fine del mutamento. Dopo aver effettuato l'acquisto occorre attendere diversi giorni prima di poter ricevere i prodotti nelle nostre abitazioni.

E' qui che trovano spazio i droni, veicoli elettrici che in autonomia possono spostarsi in volo evitando il traffico e di fatto riducendo le tempistiche per gli spostamenti di merci. Allora, data una flotta di droni, una lista di ordini e di disponibilità nei magazzini come si possono pianificare le azioni da comunicare ai veicoli per completare le consegne nel minor tempo possibile?

#### Le specifiche

La simulazione trova luogo in una griglia bidimensionale sulla quale i droni possono "volare". Ogni cella viene identificata dalla coordinata riga/colonna.

Sulla griglia trovano luogo i magazzini (ognuno con coordinate differenti) e i destinatari degli ordini.

Ogni prodotto ha un tipo, il tipo specifica il peso.

I magazzini contengono quantità definite di prodotti di diversi tipi, non per forza un magazzino contiene prodotti di tutti i tipi. Durante la simulazione i magazzini non vengono riforniti.

Gli ordini contengono le coordinate del cliente e specificano i prodotti richiesti. I prodotti richiesti possono avere tipi diversi e contenere più prodotti con lo stesso tipo.

I droni sono in grado di caricare un certo peso uguale per tutti e possono trasportare gli articoli dai magazzini ai clienti. Per i loro spostamenti percorrono sempre la strada più breve (distanza Euclidea).

I droni sono in grado di eseguire i seguenti comandi:

- **Load:**  Vola fino alla coordinata del magazzino specificato e carica un certo numero di prodotti di un certo tipo.
- **Deliver:** Vola fino alla coordinata del cliente e scarica un certo numero di prodotti di un certo tipo.

#### Input, output e valutazione dalla soluzione

Gli elementi ricevuti in input sono:

- la dimensione della griglia
- il numero di droni disponibili
- il numero massimo di turni (spiegati in seguito)
- il peso massimo trasportabile da un drone
- i magazzini (coordinate e contenuto)
- gli ordini (coordinate di consegna e prodotti che ne fanno parte)

Gli elementi da fornire in output:

- la lista delle azioni che i droni devono eseguire (lista di Load e Deliver)

Ogni comando della soluzione consuma *d*+1 turni , dove *d* è la distanza percorsa dal drone per poter soddisfare la richiesta (0 se il drone si trova già in posizione).

La valutazione della soluzione trovata è definita da un punteggio che diminuisce all'aumentare del numero di turni in cui ogni ordine viene completato. Il punteggio finale consiste nella somma dei punti ottenuti per ogni ordine che viene completato entro il limite di turni (massimo numero di turni).

Risulta chiaro che soluzioni con un punteggio migliore sullo stesso input sono vantaggiose in termini di tempo di consegna degli ordini.

##La soluzione

perché pianificazione? perché strips?

