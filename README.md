# Classical Planner with h<sub>max</sub> and LM-Cut

This project implements an optimal classical planner for deterministic planning tasks in SAS (FDR) format.

## The planner:

* Parses SAS planning problems

* Converts them into a STRIPS-like representation

* Solves them optimally using A* search

* Supports two admissible heuristics:
  
  1.  h<sub>max</sub>

  2.  LM-Cut

The implementation is written entirely in Python and does not rely on external planning libraries.

## How to run:
* ### Compute h<sub>max</sub>
  python hmax.py `<sas_file>`
* ### Compute LM-Cut
  python lmcut.py `<sas_file>`
* ### Run the Planner (A*)
  python planner.py `<sas_file>` `<heuristic>`
  Where `<heuristic>`  is one of:

  1.  hmax
     
  2.  lmcut
