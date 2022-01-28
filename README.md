# Self Driving Car Path Planning Project

## Documentation
The Old README.md file is renamed as [Readme.old.md](#Readme.old.md)

### How to use
- Download and Open the term 3 simulator. The link is provided in the old readme file.
- Run the following commands to build the binary file
```bash
cd build && cmake .. && make
```
- Then run the binary named `path_planning`
```./path_planning` 


## Reflection
The reflection on the code, i.e. the code structure is described in [Reflection.md](#Reflection.md) file

### Way of Improvement

One way of improvement is to predict the cars in each lane beforehand and choose the best lane at each iteration. Then select the behaviour at each iteration accordingly. It should optimize the path planning