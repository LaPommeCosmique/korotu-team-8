
# Installation instructions

...

# Note

This repository uses the following as submodules:
- the Tl31-prod repository for drone-sensor communication
- robopeak/rplidar_ros for scanning
- hector_slam for mapping

To clone the project with submodules, run the following command.

```
git clone --recursive git@github.com:LaPommeCosmique/korotu-team-8.git 
```

If you have already cloned the project, or you want to make sure your submodules are up to date, you can run the following command.

```
git submodule update --init --recursive
```

If you only want to update one submodule (i.e. the TL31 team updates the drone-sensor protocol), the following command can be run.

```
git submodule update <specific path to submodule>
```

Submodules were added with the following command.

```
git submodule add <submodule link> <path>

e.g.
git submodule add git submodule add git@github.com:robopeak/rplidar_ros.git src/rplidar_ros
```

**IMPORTANT NOTE: WE ARE CURRENTLY WAITING FOR Tl31-prod TO MERGE emern/drone_interface WITH THE MAIN BRANCH. WHEN THAT IS COMPLETE, DELETE THE PROTOCOL/ FOLDER AND UPDATE REFERENCES TO USE THE PROTOCOL/ FOLDER IN Tl31-prod**