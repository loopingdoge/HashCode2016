## Tests

1. generate the input
    ```bash
    # es.
    python src/scripts/generate_input.py 50 50 5 20000 250 5 5 30
    ```

2. run the comparison
    ```bash
    # es. (write the --desc by hand)
    python src/scripts/planners_comparison_log.py generated --desc 50_50_1_20000_250_5_5_30
    ```

    with this:
    - a new folder named "timestamp_description" is created under tests /
    - the input file "generated.in" is moved in the new folder
    - a log file for each planner is created in the new folder. It will contains: the moves, the profile (predicate calls and execution time), the score and the number of turns, the timestamp of the start and the end time.
    - each planner has at most 1 hour to complete
    