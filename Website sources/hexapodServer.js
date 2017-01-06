var hexapodServerHttp = require('http').createServer(handler);
var WebSocketClient = require('websocket').client;
var url= require('url')
var fs = require('fs')

var hexapodClientWebsocket = new WebSocketClient();
var hexapodServerWebsocket = require('socket.io').listen(hexapodServerHttp);

// Http handler function
function handler (req, res)
{
    // Using URL to parse the requested URL
    var path = url.parse(req.url).pathname;
    
    // Managing the root route
    if (path == '/')
	{
        index = fs.readFile	(__dirname+'/public/mainPage.html', 
            function(error,data)
			{
                if (error) {
                    res.writeHead(500);
                    return res.end("Error: unable to load mainPage.html");
                }
                res.writeHead(200,{'Content-Type': 'text/html'});
                res.end(data);
            }
							);
    }
	// Managing the route for the javascript files
	else if( /\.(js)$/.test(path) )
	{
        index = fs.readFile	(__dirname+'/public'+path, 
            function(error,data)
			{
                if (error) {
                    res.writeHead(500);
                    return res.end("Error: unable to load " + path);
                }
                res.writeHead(200,{'Content-Type': 'text/plain'});
                res.end(data);
            }
							);
    }
	// Managing the route for the css files
	else if( /\.(css)$/.test(path) )
	{
        index = fs.readFile	(__dirname+'/public'+path, 
            function(error,data)
			{
                if (error)
				{
                    res.writeHead(500);
                    return res.end("Error: unable to load " + path);
                }
                res.writeHead(200,{'Content-Type': 'text/plain'});
                res.end(data);
            }
							);
    }
	// Try open standart page
	else
	{
        index = fs.readFile	(__dirname+'/public'+path, 
            function(error,data)
			{
                if (error) {
                    res.writeHead(404);
					return res.end("Error: 404 - File not found.");
                }
                res.writeHead(200,{'Content-Type': 'text/html'});
                res.end(data);
            }
							);
    }
	
}



var clientCount = 0;
var connectedToRobot = false;
var robotConnection;

// Wifi conection
hexapodServerWebsocket.sockets.on('connection', function (socket)
{
	clientCount++;
    console.log('Wifi client connection');
	socket.emit('server', 'Welcome friend !');
	
	socket.on('server', function (msg) { console.log('in msg : ' + msg); });
	socket.on('disconnect', function () {
		clientCount--;
		console.log('Wifi client disconnected');
	});
});


// Robot connection
hexapodClientWebsocket.on('connectFailed', function(error)
{
	if(error.code != 'ECONNREFUSED')
		console.log('Connect Fail: ' + error.toString());
	connectedToRobot = false;
});
hexapodClientWebsocket.on('connect', function(connection)
{
    console.log('Hexapod WebSocket Client Connected');
	connectedToRobot = true;
	robotConnection = connection;
	hexapodServerWebsocket.sockets.emit('robot', 'connected');
	
    connection.on('error', function(error)
	{
        console.log("Robot connection error: " + error.toString());
    });
    connection.on('close', function()
	{
        console.log('Connection to robot closed');
		hexapodServerWebsocket.sockets.emit('robot', 'disconnected');
		connectedToRobot = false;
    });
    connection.on('message', function(message)
	{
		if(message.type == 'utf8')
		{
			var header = message.utf8Data.substr(0,message.utf8Data.indexOf(' '));
			var data = message.utf8Data.substr(message.utf8Data.indexOf(' ')+1);
			hexapodServerWebsocket.sockets.emit(header, data);
		}
		else
		{
			console.log(message);
		}
    });
});


//listen to port 80 : default for www or http
hexapodServerHttp.listen(80);
hexapodClientWebsocket.connect('ws://localhost:5012', 'robot-protocol');

//	refresh loop
var interval = setInterval( function()
{
  if(!connectedToRobot)
	  hexapodClientWebsocket.connect('ws://localhost:5012', 'robot-protocol');
}, 100);







