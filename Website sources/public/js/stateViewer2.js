
/*
	global attributes and variables
*/
var canvasContainer;
var myPI = 3.14159267;

var socket;

var picking = { selectedNode:-1, hoverNode:-1 };
var animation = { foldingSpeed:0.01, foldingTime:0 };
var drawingFile = null;

var textureNode = { online:null, offline:null, selected:null, hover:null, master:null };
var textureModule = { online:null, offline:null, selected:null, hover:null, master:null };
var textureAside = { online:null, offline:null, master:null };

var graphAnimState = { folding:0, foldingDirection:1, foldingTime:0.0, aside:0, asideDirection:0, asideTime:0, position:0, upDown:0, upDownDirection:0, upDownTime:0.0 };
var graphNodeAttribute = { zeroRadius:0, moduleRetract:0, branchAngle: [0,0,0,0,0,0,0,0], offsetX:0, offsetY:0 };
var drawingNodeAttribute = { border:6, radius:0, masterRadius:0, moduleBoxRatio:0, displacementFactor:0 };

var specialIDs = { arduino:0, rpi3:0, moduleFront:0, moduleBack:0 };

var messageString = null;
var serverMessage = null;

var nodeMap = {};

/*
	setup functions called once for instanciation
*/
function preload()
{
	textureNode.online =   loadImage("./picture/NodeOnline.png");
	textureNode.offline =  loadImage("./picture/NodeOffline.png");
	textureNode.hover =    loadImage("./picture/NodeHover.png");
	textureNode.selected = loadImage("./picture/NodeSelected.png");
	textureNode.master =   loadImage("./picture/NodeMaster.png");
	
	textureModule.online =   loadImage("./picture/ModuleOnline.png");
	textureModule.offline =  loadImage("./picture/ModuleOffline.png");
	textureModule.hover =    loadImage("./picture/ModuleHover.png");
	textureModule.selected = loadImage("./picture/ModuleSelected.png");
	textureModule.master =   loadImage("./picture/ModuleMaster.png");
	
	textureAside.online =   loadImage("./picture/AsideOnline.png");
	textureAside.offline =   loadImage("./picture/AsideOffline.png");
	textureAside.master =   loadImage("./picture/AsideMaster.png");
	
	if(!textureNode.online || !textureNode.offline || !textureNode.hover || !textureNode.selected || !textureNode.master) messageString = "texture node loading error";
	else if(!textureModule.online || !textureModule.offline || !textureModule.hover || !textureModule.selected || !textureModule.master) messageString = "texture module loading error";
	else if(!textureAside.online || !textureAside.offline || !textureAside.master) messageString = "texture aside loading error";
	
	drawingFile = loadJSON("./ressources/hexapodNetwork.json");
	if(!drawingFile) messageString = "json file error\nsee console log for more infos";
}
function setup()
{
	//	p5js and other stuf
	canvasContainer = document.getElementById('stateViewerWindow');
	var myCanvas = createCanvas(canvasContainer.clientWidth, canvasContainer.clientHeight);
	myCanvas.parent('stateViewerWindow');

	frameRate(30);
	colorMode(RGB, 255, 255, 255, 1);
	textSize(24);
	textAlign(CENTER, CENTER);
	
	//	drawing attributes initialization
	drawingNodeAttribute.border = drawingFile.nodeDrawing.border;
	drawingNodeAttribute.radius = drawingFile.nodeDrawing.radius;
	drawingNodeAttribute.masterRadius = drawingFile.nodeDrawing.masterRadius;
	drawingNodeAttribute.moduleBoxRatio = drawingFile.nodeDrawing.moduleBoxRatio;
	drawingNodeAttribute.displacementFactor = drawingFile.folding[0].displacementFactor;
	
	graphNodeAttribute.zeroRadius = drawingFile.folding[0].zeroRadius;
	graphNodeAttribute.moduleRetract = drawingFile.folding[0].moduleRetract;
	for (j = 0; j < 8; j++)
		graphNodeAttribute.branchAngle[j] = drawingFile.folding[0].branchAngle[j];
	
	specialIDs.arduino = drawingFile.specialIds.Arduino;
	specialIDs.rpi3 = drawingFile.specialIds.RPI3;
	
	specialIDs.moduleFront = drawingFile.specialIds.ModuleFront;
	specialIDs.moduleBack = drawingFile.specialIds.ModuleBack;
	
	//	personnal node attributes
	for (j = 0; j < 26; j++)
	{
		nodeMap[nf(j)] = { online:false, position:0, torque:0, temperature:0 };
	}
	nodeMap[specialIDs.arduino] = { online:true };
	nodeMap[specialIDs.rpi3] = { online:false };
	nodeMap[specialIDs.moduleFront] = { online:false };
	nodeMap[specialIDs.moduleBack] = { online:false };

	//	communication with server
	socket = io.connect('http://hexapod.local:80');
	socket.on('connect', function()
	{
		//	initialise server/client/robot protocol
		serverMessage = "connected";
		socket.on('server', function(msg){ serverMessage = msg; });
		socket.on('robot', function(msg)
		{
			serverMessage = 'robot ' + msg;
			if(msg === 'disconnected')
			{
				nodeMap[specialIDs.rpi3].online = false;
				for (j = 0; j < 26; j++)
					nodeMap[j].online = false;
			}
			else if(msg === 'connected') nodeMap[specialIDs.rpi3].online = true;
		});
		
		//	initialize node message protocol
		socket.on('SCS15_fail', function(msg)
		{
			var array = msg.split(" ", 26);
			if(array.length != 26) serverMessage = "error:" + msg;
			else
			{
				for (j = 0; j < 26; j++)
					nodeMap[j].online = (array[j] == "0");
			}
		});
		socket.on('SCS15_position', function(msg)
		{
			var array = msg.split(" ", 26);
			if(array.length != 26) serverMessage = "error:" + msg;
			else
			{
				for (j = 0; j < 26; j++)
					nodeMap[j].position = parseInt(array[j]);
			}
		});
		socket.on('SCS15_torque', function(msg)
		{
			var array = msg.split(" ", 26);
			if(array.length != 26) serverMessage = "error:" + msg;
			else
			{
				for (j = 0; j < 26; j++)
					nodeMap[j].torque = parseInt(array[j]);
			}
		});
		socket.on('SCS15_temperature', function(msg)
		{
			var array = msg.split(" ", 26);
			if(array.length != 26) serverMessage = "error:" + msg;
			else
			{
				for (j = 0; j < 26; j++)
					nodeMap[j].temperature = parseInt(array[j]);
			}
		});
	});
	socket.on('disconnect', function()
	{
		serverMessage = "disconnected";
		for (j = 0; j < 26; j++)
			nodeMap[j].online = false;
	});
}


/*
	callback functions : used by p5.js for user interactions
*/
function windowResized()
{
	resizeCanvas(canvasContainer.clientWidth, canvasContainer.clientHeight);
	if(graphAnimState.aside === 2)
	{
		graphNodeAttribute.offsetX = -0.5 * height * drawingFile.asideDrawing.ratio * drawingFile.asideDrawing.height;
	}
}
function mousePressed()  { if(mouseButton == LEFT) picking.selectedNode = picking.hoverNode; }


/*
	useful functions
*/
function httpGetRequest(theUrl)
{
    var xmlHttp = new XMLHttpRequest();
    xmlHttp.open( "GET", theUrl, false ); // false for synchronous request
    xmlHttp.send( null );
    return xmlHttp.responseText;
}
function drawNode(name, x, y, radius, tex)
{
	image(tex, x - radius, y - radius, 2 * radius, 2 * radius);
	text(name, x, y);
	
	if(sqrt((mouseX - x)*(mouseX - x)+(mouseY - y)*(mouseY - y)) < radius) return true;
	else return false;
}
function drawModule(name, x, y, size, tex)
{
	image(tex, x - size, y - drawingNodeAttribute.moduleBoxRatio*size, 2 * size, 2 * size * drawingNodeAttribute.moduleBoxRatio);
	text(name, x, y);
	
	// Picking test
	if(abs(mouseX-x) < size && abs(mouseY-y) < drawingNodeAttribute.moduleBoxRatio * size) return true;
	else return false;
}
function drawLink(x1, y1, x2, y2) { line(x1,y1, x2,y2); }
function drawDotLink(x1, y1, x2, y2, dotCount)
{
	for(i = 0; i <= dotCount; i++)
	{
		line( lerp(x1,x2, i/dotCount),
			  lerp(y1,y2, i/dotCount),
			  lerp(x1,x2, (i+0.3)/dotCount),
			  lerp(y1,y2, (i+0.3)/dotCount)	);
	}
}
function getNodeTexture(index)
{
	if(index === picking.selectedNode) return textureNode.selected;
	else if(index === picking.hoverNode) return textureNode.hover;
	else if(!nodeMap[index].online) return textureNode.offline;
	else
	{
		if(index === specialIDs.arduino || index === specialIDs.rpi3) return textureNode.master;
		else return textureNode.online;
	}
}
function getModuleTexture(index)
{
	if(index === picking.selectedNode) return textureModule.selected;
	else if(index === picking.hoverNode) return textureModule.hover;
	else if(!nodeMap[index].online) return textureModule.offline;
	else return textureModule.online;
}
function getAsideTexture(index)
{
	if(index < 0) return textureAside.offline;
	else if(!nodeMap[index].online)
		return textureAside.offline;
	else if(index === specialIDs.arduino || index === specialIDs.rpi3)
		return textureAside.master;
	else return textureAside.online;
}
function printNode(index, x, y)
{
	var lineOrigin = { x:25, y:75 };
	var lineHeight = 25;
	var lineCounter = 0;
	
	//	get name
	var name = "Unknown";
	if(index === specialIDs.arduino)
		name = "Arduino";
	else if(index === specialIDs.rpi3)
		name = "Raspberry";
	else if(index === specialIDs.moduleFront)
		name = "Front module";
	else if(index === specialIDs.moduleBack)
		name = "Back module";
	else if(index >= 0)
		name = "Node ID : " + nf(index);

	//	print name
	textStyle(BOLD);
	text(name, x + lineOrigin.x, y + lineOrigin.y + lineCounter * lineHeight);
	textStyle(NORMAL);
	lineCounter++;

	//	print all others attributes
	if(index < 0) return;
	else if(!nodeMap[index].online)
	{
		text("offline / disconnected", x + lineOrigin.x, y + lineOrigin.y + lineCounter * lineHeight);
		lineCounter++;
	}
	else
	{
		for (var key in nodeMap[index])
		{
			if(key === 'online') continue;
			else if (nodeMap[index].hasOwnProperty(key))
			{
				text(key + ": " + nodeMap[index][key], x + lineOrigin.x, y + lineOrigin.y + lineCounter * lineHeight);
				lineCounter++;
			}
		}
	}
}


/*
	draw functions
*/
//var centerx,centery, px0,py0, dirx,diry;
function drawHexapodLinks()
{
	// initialization
	stroke(0);
	strokeWeight(drawingNodeAttribute.border);
	
	var centerx = width/2  + graphNodeAttribute.offsetX;
	var centery = height/2 + graphNodeAttribute.offsetY;
	var px0 = centerx;
	var py0 = centery;
	var dirx = cos(graphNodeAttribute.branchAngle[7]);
	var diry = sin(graphNodeAttribute.branchAngle[7]);

	var d = graphNodeAttribute.zeroRadius + 3.5 * drawingNodeAttribute.displacementFactor * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;
	var px1 = centerx + d*dirx;
	var py1 = centery + d*diry;

	// Arduino / RPI
	drawDotLink(
		px0 + drawingNodeAttribute.masterRadius * dirx,	// extremity X 0
		py0 + drawingNodeAttribute.masterRadius * diry,	// extremity Y 0
		px1 - 1.5 * drawingNodeAttribute.radius * dirx,	// extremity X 1
		py1 - 1.5 * drawingNodeAttribute.radius * diry,	// extremity Y 1
		8   );											// dot count

	dirx = cos(graphNodeAttribute.branchAngle[6]);
	diry = sin(graphNodeAttribute.branchAngle[6]);
	d = graphNodeAttribute.zeroRadius + 2.5 * drawingNodeAttribute.displacementFactor * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;
	px1 = centerx + d*dirx;
	py1 = centery + d*diry;

	// node 0 / Arduino
	drawLink(
		px0 + drawingNodeAttribute.masterRadius * dirx,
		py0 + drawingNodeAttribute.masterRadius * diry,
		px1 - drawingNodeAttribute.radius * dirx,
		py1 - drawingNodeAttribute.radius * diry   );

	d = graphNodeAttribute.zeroRadius + 3.5 * drawingNodeAttribute.displacementFactor * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;
	px0 = px1;   px1 = centerx + d*dirx;
	py0 = py1;   py1 = centery + d*diry;

	// node 0 / Arduino
	drawLink(
		px0 + drawingNodeAttribute.radius * dirx,
		py0 + drawingNodeAttribute.radius * diry,
		px1 - drawingNodeAttribute.radius * dirx,
		py1 - drawingNodeAttribute.radius * diry   );

	// Draw All Legs net link
	for (j = 0; j < 6; j++)
	{
		dirx = cos(graphNodeAttribute.branchAngle[j]);
		diry = sin(graphNodeAttribute.branchAngle[j]);

		for (i = 0; i < 4; i++)
		{
			d = graphNodeAttribute.zeroRadius + (i + 1) * drawingNodeAttribute.displacementFactor * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;
			
			px1 = centerx + d*dirx;
			py1 = centery + d*diry;
			
			if(i === 0)
			{
				px0 = centerx;
			 	py0 = centery;
				
				drawLink(
					px0 + drawingNodeAttribute.masterRadius * dirx,
					py0 + drawingNodeAttribute.masterRadius * diry,
					px1 - drawingNodeAttribute.radius * dirx,
					py1 - drawingNodeAttribute.radius * diry   );
			}
			else
			{
				drawLink(
					px0 + drawingNodeAttribute.radius * dirx,
					py0 + drawingNodeAttribute.radius * diry,
					px1 - drawingNodeAttribute.radius * dirx,
					py1 - drawingNodeAttribute.radius * diry   );
			}
			
			px0 = px1;
			py0 = py1;
		}
	}
	
	// Module net link
	py1 = centery - graphNodeAttribute.moduleRetract + 8 * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;
	drawLink(
		centerx,
		centery + drawingNodeAttribute.masterRadius,
		centerx,
		py1 - 2 * drawingNodeAttribute.moduleBoxRatio * drawingNodeAttribute.radius   );		

	py1 = centery + graphNodeAttribute.moduleRetract - 8 * drawingNodeAttribute.radius - drawingNodeAttribute.masterRadius/2;
	drawLink(
		centerx,
		centery - drawingNodeAttribute.masterRadius,
		centerx,
		py1 + 2 * drawingNodeAttribute.moduleBoxRatio * drawingNodeAttribute.radius   );		
}
function drawHexapodNodes()
{
	// initialization
	fill(0);
	noStroke();
	
	var tmpHoverNode = -1;
	var centerx = width/2  + graphNodeAttribute.offsetX;
	var centery = height/2 + graphNodeAttribute.offsetY;
	
	
	// Node Arduino
	if( drawNode(
			"Arduino",							// node name
			centerx,							// position X
			centery,							// position Y
			drawingNodeAttribute.masterRadius,	// radius
			getNodeTexture(specialIDs.arduino)   )				// computed color
	  ) tmpHoverNode = specialIDs.arduino;						// if hoved by cursor update hover index
	
	var dirx = cos(graphNodeAttribute.branchAngle[7]);
	var diry = sin(graphNodeAttribute.branchAngle[7]);
	var d = graphNodeAttribute.zeroRadius + 3.5 * drawingNodeAttribute.displacementFactor * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;
	
	// Node RPI
	if( drawNode(
			"RPI3",
			centerx + d*dirx,
			centery + d*diry,
			0.65 * drawingNodeAttribute.masterRadius,
			getNodeTexture(specialIDs.rpi3)   )
	  ) tmpHoverNode = specialIDs.rpi3;
	
	dirx = cos(graphNodeAttribute.branchAngle[6]);
	diry = sin(graphNodeAttribute.branchAngle[6]);
	d = graphNodeAttribute.zeroRadius + 2.5 * drawingNodeAttribute.displacementFactor * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;

	// Node 0
	if( drawNode(
			nf(0),
			centerx + d*dirx,
			centery + d*diry,
			drawingNodeAttribute.radius,
			getNodeTexture(0)   )
	  ) tmpHoverNode = 0;
	
	d = graphNodeAttribute.zeroRadius + 3.5 * drawingNodeAttribute.displacementFactor * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;

	// Node 1
	if( drawNode(
			nf(1),
			centerx + d*dirx,
			centery + d*diry,
			drawingNodeAttribute.radius,
			getNodeTexture(1)   )
	  ) tmpHoverNode = 1;
	
	// draw all Hexapod Nodes
	for (j = 0; j < 6; j++)
	{
		dirx = cos(graphNodeAttribute.branchAngle[j]);
		diry = sin(graphNodeAttribute.branchAngle[j]);
		
		for (i = 0; i < 4; i++)
		{
			var name = 4 * j + i + 2;
			d = graphNodeAttribute.zeroRadius + (i + 1) * drawingNodeAttribute.displacementFactor * drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2;
			
			// Node "name"
			if( drawNode(
					nf(name),
					centerx + d*dirx,
					centery + d*diry,
					drawingNodeAttribute.radius,
					getNodeTexture(name)   )
			  ) tmpHoverNode = name;
		}
	}

	if(graphAnimState.upDown != 4)
	{
		if( drawModule(
				"Back\nModule",
				centerx,
				centery - graphNodeAttribute.moduleRetract + 8*drawingNodeAttribute.radius + drawingNodeAttribute.masterRadius/2,
				2*drawingNodeAttribute.radius,
				getModuleTexture(specialIDs.moduleBack) )
		  ) tmpHoverNode = specialIDs.moduleBack;			
	}
	if(graphAnimState.upDown != 2)
	{
		if( drawModule(
				"Front\nModule",
				centerx,
				centery + graphNodeAttribute.moduleRetract - 8*drawingNodeAttribute.radius - drawingNodeAttribute.masterRadius/2,
				2*drawingNodeAttribute.radius,
				getModuleTexture(specialIDs.moduleFront) )
		  ) tmpHoverNode = specialIDs.moduleFront;			
	}
	
	
	picking.hoverNode = tmpHoverNode;
}
function drawAside()
{
	if(graphAnimState.aside === 0) return;
	textAlign(LEFT, CENTER);
	
	// panel position, panel size, line parameters
	var sx = height * (drawingFile.asideDrawing.ratio * drawingFile.asideDrawing.height - 2 * drawingFile.asideDrawing.margin);
	var sy = height * (drawingFile.asideDrawing.height - 2 * drawingFile.asideDrawing.margin);
	var x = width + height * drawingFile.asideDrawing.ratio * drawingFile.asideDrawing.height * (0.5 - 1.0 * graphAnimState.asideTime);
	var y = height / 2;
	var lineHeight = 25;
	var line = 0;

	// 
	image(getAsideTexture(picking.selectedNode), x - sx / 2, y - sy / 2, sx, sy);
	printNode(picking.selectedNode, x - sx / 2, y - sy / 2)
	
	// picking
	if(abs(mouseX-x) < sx/2 && abs(mouseY-y) < sy/2)
		picking.hoverNode = picking.selectedNode;
	
	// end
	textAlign(CENTER, CENTER);
}


/*
	animation graph functions
*/
var animationStep = 0.03;
function animateFolding()
{
	// handle folding animation state
	if(graphAnimState.folding === 0)
	{
		if(nodeMap[specialIDs.moduleFront].online && picking.selectedNode === specialIDs.moduleFront) // start folding
		{
			graphAnimState.foldingTime = 0.0;
			graphAnimState.folding = 1;
			graphAnimState.foldingDirection = 1;
			frameRate(60);
		}
		else if(nodeMap[specialIDs.moduleBack].online && picking.selectedNode === specialIDs.moduleBack) // start folding
		{
			graphAnimState.foldingTime = 0.0;
			graphAnimState.folding = 1;
			graphAnimState.foldingDirection = 1;
			frameRate(60);
		}
	}
	else if(graphAnimState.folding === 1)
	{
		graphAnimState.foldingTime += graphAnimState.foldingDirection*animationStep;
		if(graphAnimState.foldingDirection > 0 && graphAnimState.foldingTime >= 1.0)
		{
			graphAnimState.foldingTime = 1.0;
			graphAnimState.folding = 2;
			frameRate(30);
		}
		else if(graphAnimState.foldingDirection < 0 && graphAnimState.foldingTime <= 0.0)
		{
			graphAnimState.foldingTime = 0.0;
			graphAnimState.folding = 0;
			frameRate(30);
		}
		
		for (j = 0; j < 8; j++)
			graphNodeAttribute.branchAngle[j] = (1.0 - graphAnimState.foldingTime) * drawingFile.folding[0].branchAngle[j] + 
												graphAnimState.foldingTime * drawingFile.folding[1].branchAngle[j];
		
		graphNodeAttribute.zeroRadius = (1.0 - graphAnimState.foldingTime) * drawingFile.folding[0].zeroRadius + 
										graphAnimState.foldingTime * drawingFile.folding[1].zeroRadius;
										
		graphNodeAttribute.moduleRetract = (1.0 - graphAnimState.foldingTime) * drawingFile.folding[0].moduleRetract + 
										   graphAnimState.foldingTime * drawingFile.folding[1].moduleRetract;
												  
		drawingNodeAttribute.displacementFactor = (1.0 - graphAnimState.foldingTime) * drawingFile.folding[0].displacementFactor + 
												  graphAnimState.foldingTime * drawingFile.folding[1].displacementFactor;
	}
	else if(graphAnimState.folding === 2)
	{
		if(picking.selectedNode >= 0 && picking.selectedNode < 26) // start unfolding
		{
			graphAnimState.foldingTime = 1.0;
			graphAnimState.folding = 1;
			graphAnimState.foldingDirection = -1;
			frameRate(60);
		}
		else if(picking.selectedNode === -1 || picking.selectedNode === specialIDs.arduino || picking.selectedNode === specialIDs.rpi3) // start unfolding
		{
			graphAnimState.foldingTime = 1.0;
			graphAnimState.folding = 1;
			graphAnimState.foldingDirection = -1;
			frameRate(60);
		}
	}
}
function animateAside()
{
	// handle aside animation state
	if(graphAnimState.aside === 0)
	{
		if(nodeMap[specialIDs.moduleFront].online && picking.selectedNode === specialIDs.moduleFront) return;
		else if(nodeMap[specialIDs.moduleBack].online && picking.selectedNode === specialIDs.moduleBack) return;
		else if(picking.selectedNode >= 0)
		{
			graphAnimState.asideTime = 0.0;
			graphAnimState.aside = 1;
			graphAnimState.asideDirection = 1;
			frameRate(60);
		}
	}
	else if(graphAnimState.aside === 1)
	{
		graphAnimState.asideTime += graphAnimState.asideDirection * animationStep;
		if(graphAnimState.asideDirection > 0 && graphAnimState.asideTime >= 1.0)
		{
			graphAnimState.asideTime = 1.0;
			graphAnimState.aside = 2;
			frameRate(30);
		}
		else if(graphAnimState.asideDirection < 0 && graphAnimState.asideTime <= 0.0)
		{
			graphAnimState.asideTime = 0.0;
			graphAnimState.aside = 0;
			frameRate(30);
		}
		
		graphNodeAttribute.offsetX = -0.5 * graphAnimState.asideTime * height * drawingFile.asideDrawing.ratio * drawingFile.asideDrawing.height;
	}
	else if(graphAnimState.aside === 2)
	{
		if(picking.selectedNode < 0)
		{
			graphAnimState.asideTime = 1.0;
			graphAnimState.aside = 1;
			graphAnimState.asideDirection = -1;
			frameRate(60);
		}
		else if(nodeMap[specialIDs.moduleFront].online && picking.selectedNode === specialIDs.moduleFront)
		{
			graphAnimState.asideTime = 1.0;
			graphAnimState.aside = 1;
			graphAnimState.asideDirection = -1;
			frameRate(60);
		}
		else if(nodeMap[specialIDs.moduleBack].online && picking.selectedNode === specialIDs.moduleBack)
		{
			graphAnimState.asideTime = 1.0;
			graphAnimState.aside = 1;
			graphAnimState.asideDirection = -1;
			frameRate(60);
		}
	}
}
function animateUpDown()
{
	// handle up/down animation state
	if(graphAnimState.upDown === 0)
	{
		if(nodeMap[specialIDs.moduleFront].online && picking.selectedNode === specialIDs.moduleFront)
		{
			graphAnimState.upDownTime = 0.0;
			graphAnimState.upDown = 1;
			graphAnimState.upDownDirection = 1;
			frameRate(60);
		}
		else if(nodeMap[specialIDs.moduleBack].online && picking.selectedNode === specialIDs.moduleBack)
		{
			graphAnimState.upDownTime = 0.0;
			graphAnimState.upDown = 3;
			graphAnimState.upDownDirection = 1;
			frameRate(60);
		}
	}
	
	//	go down / go mid
	else if(graphAnimState.upDown === 1)
	{
		graphAnimState.upDownTime += graphAnimState.upDownDirection * animationStep;
		if(graphAnimState.upDownDirection > 0 && graphAnimState.upDownTime >= 1.0)
		{
			graphAnimState.upDownTime = 1.0;
			graphAnimState.upDown = 2;
			frameRate(30);
		}
		else if(graphAnimState.upDownDirection < 0 && graphAnimState.upDownTime <= 0.0)
		{
			graphAnimState.upDownTime = 0.0;
			graphAnimState.upDown = 0;
			frameRate(30);
		}
		
		graphNodeAttribute.offsetY = graphAnimState.upDownTime * height / 4.0;
	}
	else if(graphAnimState.upDown === 2)
	{
		if(picking.selectedNode != specialIDs.moduleFront)
		{
			graphAnimState.upDownTime = 1.0;
			graphAnimState.upDown = 1;
			graphAnimState.upDownDirection = -1;
			frameRate(60);
		}
	}
	
	//	go up / go mid
	else if(graphAnimState.upDown === 3)
	{
		graphAnimState.upDownTime += graphAnimState.upDownDirection * animationStep;
		if(graphAnimState.upDownDirection > 0 && graphAnimState.upDownTime >= 1.0)
		{
			graphAnimState.upDownTime = 1.0;
			graphAnimState.upDown = 4;
			frameRate(30);
		}
		else if(graphAnimState.upDownDirection < 0 && graphAnimState.upDownTime <= 0.0)
		{
			graphAnimState.upDownTime = 0.0;
			graphAnimState.upDown = 0;
			frameRate(30);
		}
		
		graphNodeAttribute.offsetY = -graphAnimState.upDownTime * height / 4.0;
	}
	else if(graphAnimState.upDown === 4)
	{
		if(picking.selectedNode != specialIDs.moduleBack)
		{
			graphAnimState.upDownTime = 1.0;
			graphAnimState.upDown = 3;
			graphAnimState.upDownDirection = -1;
			frameRate(60);
		}
	}
}


/*
	draw function called once per frame : 30 times by seconds
*/
function draw()
{
	// Begin frame
	clear();
	background(200);
	
	if(messageString)
	{
		text(messageString, width/2, height/2);
	}
	else
	{
		// animate graph
		animateFolding();
		animateAside();
		animateUpDown();
		
		// draw Hexapod node and links
		drawHexapodLinks();
		drawHexapodNodes();		
		
		// draw aside panel
		drawAside();
		
		if(serverMessage)
		{
			textAlign(LEFT, CENTER);
			text("Server message : " + serverMessage, 25, height - 25);
			textAlign(CENTER, CENTER);
		}
	}
}



