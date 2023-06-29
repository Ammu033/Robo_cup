# LindIMP UI

Web interface for Chrome web browser.



## Requisites

 Make sure `rosbridge` is running
	`roslaunch rosbridge_server rosbridge_websocket.launch`


## Run UI

* In a terminal :
	 `roscd lindimp_ui/www`
	 `python -m SimpleHTTPServer`

* In Chrome web browser go to :
	 `http://localhost:8000/index.html`


>> If there is a connection error:
 `WebSocket connection to 'ws://10.82.0.70:9090/' failed: Error in connection establishment: net::ERR_CONNECTION_TIMED_OUT`
	check the IP in `index.html line 293 ` >> ` ros.connect('ws://10.82.0.70:9090');`
 

## Usage

To add a new Service (and its new button on main page), add the files in the folder `www/services` and modify the file `services.json`

