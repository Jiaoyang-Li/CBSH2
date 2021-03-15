# CBSH2
 Improved Heuristics for Multi-Agent Path Finding with Conflict-Based Search [1].
 
 A more recent implementation with more CBS improvement techniques can be fonud here: https://github.com/Jiaoyang-Li/CBSH2-RTC
 
 The main goal is to improve heuristics for Conflict-Based Search by reasoning about pairwise dependencies between agents. 
 The code also contains the rectangle-symmetry reasoning technique RM from [2].
 
 The code requires the external library BOOST (https://www.boost.org/). After you installed BOOST and downloaded the source code, go into the directory of the source code and compile the code with CMake: 
```
cmake .
make
```

Then, you are able to run the code:
```
./CBSH2 -m instances/lak503d.map -a instances/lak503dmap-100agents-2.agents -o test.csv -t 60 -s 1 -h WDG -r 1
```

You can find details and explanations for all parameters with:
```
./CBSH2 --help
```

## License
 CBSH2 is released under USC – Research License. See license.md for further details.
 
## References
[1] Jiaoyang Li, Eli Boyarski, Ariel Felner, Hang Ma and Sven Koenig. Improved Heuristics for Multi-Agent Path Finding with Conflict-Based Search. In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), pages 442-449, 2019.

[2] Jiaoyang Li, Daniel Harabor, Peter Stuckey, Hang Ma and Sven Koenig. Symmetry-Breaking Constraints for Grid-Based Multi-Agent Path Finding. In Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), pages 6087-6095, 2019.
