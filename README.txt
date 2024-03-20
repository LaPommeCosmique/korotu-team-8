
# Installation instructions

...

# Note

This repository uses the Tl31-prod repository as a submodule drone-sensor communication. To clone the project with submodules, run the following command.

```
git clone --recurse-submodules git@github.com:LaPommeCosmique/korotu-team-8.git 
```

If you have already cloned the project, you can run 

```
git submodule init
git submodule update
```

If there have been changes made to the TL31-prod repository that needs to be updated on this repository, the following command can be run.

```
git submodule update --remote
```