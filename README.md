## Login
User: Watchplant  
Pass: ***********

## Docker
`docker start -i wp_demo`

## Pokretanje tmuxinatora
`tmuxinator start -p launch/only_local.yml`

## Navigacija u tmuxinatoru
Kroz tabove: `shift + strelice`  
Kroz panele: `ctrl + strelice`  
Ubij sve: `ctrl b + k`

## Pokretanje glavnog programa
`roslaunch watchplant_demo main.launch`

## Pokretanje plotjugglera
`rosrun plotjuggler plotjuggler -l plotjuggler.xml`

## Topic za plotanje
`/local/plant_data`
