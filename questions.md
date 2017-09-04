(in *italics* le mie osservazioni/risposte)

Con quali tecniche è possibile risolvere il problema e perché abbiamo selezionato il **classical planning**?

*Perché la soluzione corrisponde direttamente alle sequenze di azioni da fornire ad ogni drone*

Il problema ha molte soluzioni, ognuna con un punteggio più o meno alto. 

Perché non cerchiamo la migliore? Problemi di tempo? di complessità?

Ci va sempre bene la prima trovata dal planner? Abbiamo dato caratteristiche specifiche al planner che ci fanno andare bene la prima trovata? 

*Il nostro impegno è nel costruire il miglior planner possibile per ottenere soluzioni accettabili*



### (pag. 42) Proprietà dell'ambiente:

lavoriamo in un ambiente **completamente osservabile**?

> Fully observable environments are convenient because the agent need not to maintain any internal state to keep track of the world.

*L'agente conosce sempre le proprietà dello stato corrente*.

Deterministic or *nondeterministic*?

Episodic or *Sequential*?

Static or *dynamic*?



------

###(pag. 52) Goal-based agents:

*Il nostro è un goal-based agent, giusto?*

Definire una **utility function**?



------

### (pag. 64) Solving problems by searching

*Uninformed* or Informed? (Informed: given some guidance on where to look for solution)

*Il nostro agente è uninformed, giusto?*



--------

### (pag. 81) Tipo di ricerca della nostra soluzione:

breadth-first search (FIFO frontier) or Depth-first seearch (LIFO frontier)?

credo che la nostra sia *FIFO*, soluzione sempre trovata (se esiste) per problemi piccoli (problema state-explosion).



---------------

### (pag. 206) Stiamo risolvendo un constraint satisfation problem?

> We use a factored representation for each state: a set of variables, each of which has a value. The main idea is to eliminate large portions of the search space all at once by identifying variable/value combinations that violate the constraints.

Forse al posto di vincoli sarebbe corretto esprimere dei **preference constraints** (es. preferire drone più vicino alla warehouse)



------

### (pag. 380) Forward state-space inefficient to be practical

E' possibile una soluzione backward?

> Backward search works only when we know how to regress from state description to the predecessor state description.