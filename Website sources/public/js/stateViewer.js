var canvasContainer;
var myPI = 3.14159267;


var branchAngle = [-myPI/3, 0, myPI/3, -2*myPI/3, myPI, 2*myPI/3, -5*myPI/6, -myPI/6];; 				 // array, value can change dynamically for annimation (init in Hexapod)
var branchAngleHexapodView = [-myPI/3, 0, myPI/3, -2*myPI/3, myPI, 2*myPI/3, -5*myPI/6, -myPI/6];
var branchAngleModuleView = [-myPI/8, 0, myPI/8, -7*myPI/8, myPI, 7*myPI/8, -15*myPI/16, -myPI/16];

var selectedNode = -1; var hoverNode = -1;
var loopCounter = 0;

var graphAttribute = {posX:0, posY:0, zeroRadius:0, viewMode:0, asideState:0, moduleRetract:0};
var nodeDrawAttribute = {nodeBorder:6, nodeRadius:30, masterRadius:60, moduleBoxRatio:0.7 };


function setup()
{
	canvasContainer = document.getElementById('stateViewerWindow');
	var myCanvas = createCanvas(canvasContainer.clientWidth, canvasContainer.clientHeight);
	myCanvas.parent('stateViewerWindow');
	frameRate(30);
	colorMode(RGB, 255, 255, 255, 1);
	textSize(24);
	textAlign(CENTER, CENTER);
}




function windowResized() {
  resizeCanvas(canvasContainer.clientWidth, canvasContainer.clientHeight);
}


function drawNode(name, x,y, r,border, colorStr)
{
	var c = color(colorStr);
	stroke(red(c), green(c), blue(c), 1);
	strokeWeight(border);
	fill(colorStr);
	ellipse(x,y,r,r);
	
	fill(0);
	strokeWeight(0);
	text(name, x,y);
	
	var v = createVector(mouseX-x,mouseY-y);
	if(v.mag() < r/2) return true;
	else return false;
}

function drawModule(name, x,y, r,border, colorStr)
{
	var c = color(colorStr);
	rectMode(CENTER);
	stroke(red(c), green(c), blue(c), 1);
	strokeWeight(border);
	fill(colorStr);
	rect(x,y, r,nodeDrawAttribute.moduleBoxRatio*r, 2*border);
	
	fill(0);
	strokeWeight(0);
	text(name, x,y);
	
	var v = createVector(mouseX-x,mouseY-y);
	if(abs(mouseX-x) < r/2 && abs(mouseY-y) < r/2) return true;
	else return false;
}

function drawNetLink(x1,y1, x2,y2, border)
{
	stroke(0);
	strokeWeight(border);
	line(x1,y1, x2,y2);	
}

function drawDottedNetLink(x1,y1, x2,y2, border, dot)
{
	var dmax = createVector(x2-x1, y2-y1).mag();
	stroke(0);
	strokeWeight(border);
	
	for(i = 0; i < dot; i++)
	{
		var x3 = lerp(x1,x2, i/dot);
		var y3 = lerp(y1,y2, i/dot);
		
		var x4 = lerp(x1,x2, (i+0.5)/dot);
		var y4 = lerp(y1,y2, (i+0.5)/dot);
		
		line(x3,y3, x4,y4);
	}
}

function getNodeColor(index)
{
	if(index === selectedNode) return 'rgba(0, 128, 0, 0.5)';
	else if(index === hoverNode) return 'rgba(255, 255, 0, 0.5)';
	else
	{
		if(index === 256 || index === 257) return 'rgba(255, 0, 0, 0.5)';
		else return 'rgba(128, 0, 0, 0.5)';
	}
}

function drawAllNets()
{
	var centerx = width/2  + graphAttribute.posX;
	var centery = height/2 + graphAttribute.posY;
	
	// Arduino -> RPI serial link
	var px0 = centerx;
	var py0 = centery;
	var dirx = cos(branchAngle[7]);
	var diry = sin(branchAngle[7]);
	
	var d = graphAttribute.zeroRadius + 8.75 * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
	var px1 = centerx + d*dirx;
	var	py1 = centery + d*diry;
	
	drawDottedNetLink(	px0 + nodeDrawAttribute.masterRadius*dirx, 		py0 + nodeDrawAttribute.masterRadius*diry,
						px1 - 1.5*nodeDrawAttribute.nodeRadius*dirx, 	py1 - 1.5*nodeDrawAttribute.nodeRadius*diry,
						nodeDrawAttribute.nodeBorder, 8);
	
	// Special node 0 to 1
	px0 = centerx;
	py0 = centery;
	dirx = cos(branchAngle[6]);
	diry = sin(branchAngle[6]);
	
	d = graphAttribute.zeroRadius + 6.25 * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
	px1 = centerx + d*dirx;
	py1 = centery + d*diry;
	
	drawNetLink(px0 + nodeDrawAttribute.masterRadius*dirx,	py0 + nodeDrawAttribute.masterRadius*diry,
				px1 - nodeDrawAttribute.nodeRadius*dirx,	py1 - nodeDrawAttribute.nodeRadius*diry,
				nodeDrawAttribute.nodeBorder);
	
	d = graphAttribute.zeroRadius + 8.75 * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
	px0 = px1;
	py0 = py1;
	px1 = centerx + d*dirx;
	py1 = centery + d*diry;
	
	drawNetLink(px0 + nodeDrawAttribute.nodeRadius*dirx, py0 + nodeDrawAttribute.nodeRadius*diry,
				px1 - nodeDrawAttribute.nodeRadius*dirx, py1 - nodeDrawAttribute.nodeRadius*diry,
				nodeDrawAttribute.nodeBorder);
	
	
	// Draw All Legs net link
	for (j = 0; j < 6; j++)
	{
		dirx = cos(branchAngle[j]);
		diry = sin(branchAngle[j]);
			
		for (i = 0; i < 4; i++)
		{
			d = graphAttribute.zeroRadius + 2.5 * (i+1) * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
			
			px1 = centerx + d*dirx;
			py1 = centery + d*diry;
			
			if(i == 0)
			{
				px0 = centerx;
			 	py0 = centery;
				
				drawNetLink(px0 + nodeDrawAttribute.masterRadius*dirx,	py0 + nodeDrawAttribute.masterRadius*diry,
							px1 - nodeDrawAttribute.nodeRadius*dirx, 	py1 - nodeDrawAttribute.nodeRadius*diry,
							nodeDrawAttribute.nodeBorder);
			}
			else
			{
				drawNetLink(px0 + nodeDrawAttribute.nodeRadius*dirx, py0 + nodeDrawAttribute.nodeRadius*diry,
							px1 - nodeDrawAttribute.nodeRadius*dirx, py1 - nodeDrawAttribute.nodeRadius*diry,
							nodeDrawAttribute.nodeBorder);
			}
			
			px0 = px1;
			py0 = py1;
		}
	}
	
	// Module net link
	py1 = centery - graphAttribute.moduleRetract + 8 * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
	drawNetLink(centerx,centery + nodeDrawAttribute.masterRadius,
				centerx,py1 - 2*nodeDrawAttribute.moduleBoxRatio*nodeDrawAttribute.nodeRadius,
				nodeDrawAttribute.nodeBorder);
				
	py1 = centery + graphAttribute.moduleRetract - 8 * nodeDrawAttribute.nodeRadius - nodeDrawAttribute.masterRadius/2;
	drawNetLink(centerx,centery - nodeDrawAttribute.masterRadius,
				centerx,py1 + 2*nodeDrawAttribute.moduleBoxRatio*nodeDrawAttribute.nodeRadius,
				nodeDrawAttribute.nodeBorder);
}

function drawAllNodes()
{
	tmpHoverNode = -1;
	
	// Node Arduino
	var centerx = width/2  + graphAttribute.posX;
	var centery = height/2 + graphAttribute.posY;
	if(drawNode("Arduino", centerx,centery,
				2*nodeDrawAttribute.masterRadius,nodeDrawAttribute.nodeBorder,
				getNodeColor(256))) tmpHoverNode = 256;
	
	
	// Node 0
	var dirx = cos(branchAngle[6]);
	var diry = sin(branchAngle[6]);
	var d = graphAttribute.zeroRadius + 6.25 * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
	var px0 = centerx + d*dirx;
	var	py0 = centery + d*diry;
	
	if(drawNode(nf(0), px0,py0,
				2*nodeDrawAttribute.nodeRadius, nodeDrawAttribute.nodeBorder,
				getNodeColor(0))) tmpHoverNode = 0;
	
	// Node 1
	d = graphAttribute.zeroRadius + 8.75 * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
	px0 = centerx + d*dirx;
	py0 = centery + d*diry;
	
	if(drawNode(nf(1), px0,py0,
				2*nodeDrawAttribute.nodeRadius, nodeDrawAttribute.nodeBorder,
				getNodeColor(1))) tmpHoverNode = 1;
	
	
	// Node RPI
	dirx = cos(branchAngle[7]);
	diry = sin(branchAngle[7]);
	d = graphAttribute.zeroRadius + 8.75 * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
	px0 = centerx + d*dirx;
	py0 = centery + d*diry;
	
	if(drawNode("RPI3", px0,py0, 3*nodeDrawAttribute.nodeRadius, nodeDrawAttribute.nodeBorder, getNodeColor(257)))
		tmpHoverNode = 257;
		
	
	// Legs nodes
	for (j = 0; j < 6; j++)
	{
		dirx = cos(branchAngle[j]);
		diry = sin(branchAngle[j]);
		
		for (i = 0; i < 4; i++)
		{
			d = graphAttribute.zeroRadius + 2.5 * (i+1) * nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
			
			px0 = centerx + d*dirx;
			py0 = centery + d*diry;
			
			if(drawNode(nf(4*j+i+2), px0,py0,
					 2*nodeDrawAttribute.nodeRadius, nodeDrawAttribute.nodeBorder,
					 getNodeColor(4*j+i+2))) tmpHoverNode = 4*j+i+2;
		}
	}
	
	// Draw modules
	if(graphAttribute.viewMode != 5)
	{
		px0 = centerx;
		py0 = centery - graphAttribute.moduleRetract + 8*nodeDrawAttribute.nodeRadius + nodeDrawAttribute.masterRadius/2;
		if(drawModule("Back\nModule", px0,py0,
				   4*nodeDrawAttribute.nodeRadius, nodeDrawAttribute.nodeBorder,
				   getNodeColor(258))) tmpHoverNode = 258;
	}
	if(graphAttribute.viewMode != 2)
	{
		px0 = centerx;
		py0 = centery + graphAttribute.moduleRetract - 8*nodeDrawAttribute.nodeRadius - nodeDrawAttribute.masterRadius/2;
		if(drawModule("Front\nModule", px0,py0,
				   4*nodeDrawAttribute.nodeRadius, nodeDrawAttribute.nodeBorder,
				   getNodeColor(259))) tmpHoverNode = 259;
	}
	
	// Picking result analysis
	hoverNode = tmpHoverNode;
}



var animationTime;
var animationAsideTime;
var animationStep = 0.1;

function updateGraphView()
{
	/*
		Managment for the Hexapod default view
	*/
	if(graphAttribute.viewMode == 0) // Mode Hexapod view
	{
		if(selectedNode == 259)
		{
			animationTime = 0.0;
			graphAttribute.viewMode = 1;
		}
		else if(selectedNode == 258)
		{
			animationTime = 0.0;
			graphAttribute.viewMode = 4;
		}
	}
	
	
	
	/*
		Managment for the Hexapod module front view : stable state and animation in/out
	*/
	else if(graphAttribute.viewMode == 1) // Mode animation from Hexapod to Front view
	{
		animationTime += animationStep;
		if(animationTime >= 1.0)
		{
			animationTime = 1.0;
			graphAttribute.viewMode = 2;
		}
		
		for(i=0; i<8; i++)
			branchAngle[i] = (1.0-animationTime)*branchAngleHexapodView[i] + animationTime*branchAngleModuleView[i];
		graphAttribute.zeroRadius = animationTime*nodeDrawAttribute.masterRadius;
		graphAttribute.moduleRetract = animationTime*5*nodeDrawAttribute.nodeRadius;
		graphAttribute.posY = animationTime*height/4;
	}
	else if(graphAttribute.viewMode == 2) // Mode Module front
	{
		if(selectedNode >= 0 && selectedNode < 26)
		{
			animationTime = 1.0;
			graphAttribute.viewMode = 3;
		}
		else if(selectedNode == 258 || selectedNode == 256 || selectedNode == 257)
		{
			animationTime = 1.0;
			graphAttribute.viewMode = 3;
		}
	}
	else if(graphAttribute.viewMode == 3) // Mode animation from Front view to Hexapod
	{
		animationTime -= animationStep;
		if(animationTime <= 0.0)
		{
			animationTime = 0.0;
			graphAttribute.viewMode = 0;
		}
		
		for(i=0; i<8; i++)
			branchAngle[i] = (1.0-animationTime)*branchAngleHexapodView[i] + animationTime*branchAngleModuleView[i];
		graphAttribute.zeroRadius = animationTime*nodeDrawAttribute.masterRadius;
		graphAttribute.moduleRetract = animationTime*5*nodeDrawAttribute.nodeRadius;
		graphAttribute.posY = animationTime*height/4;
	}
	
	
	
	
	
	/*
		Managment for the Hexapod module back view : stable state and animation in/out
	*/
	else if(graphAttribute.viewMode == 4) // Mode animation from Hexapod to Back view
	{
		animationTime += animationStep;
		if(animationTime >= 1.0)
		{
			animationTime = 1.0;
			graphAttribute.viewMode = 5;
		}
		
		for(i=0; i<8; i++)
			branchAngle[i] = (1.0-animationTime)*branchAngleHexapodView[i] + animationTime*branchAngleModuleView[i];
		graphAttribute.zeroRadius = animationTime*nodeDrawAttribute.masterRadius;
		graphAttribute.moduleRetract = animationTime*5*nodeDrawAttribute.nodeRadius;
		graphAttribute.posY = - animationTime*height/4;
	}
	else if(graphAttribute.viewMode == 5) // Mode Module back
	{
		if(selectedNode >= 0 && selectedNode < 26)
		{
			animationTime = 1.0;
			graphAttribute.viewMode = 6;
		}
		else if(selectedNode == 259 || selectedNode == 256 || selectedNode == 257)
		{
			animationTime = 1.0;
			graphAttribute.viewMode = 6;
		}
	}
	else if(graphAttribute.viewMode == 6) // Mode animation from Back view to Hexapod
	{
		animationTime -= animationStep;
		if(animationTime <= 0.0)
		{
			animationTime = 0.0;
			graphAttribute.viewMode = 0;
		}
		
		for(i=0; i<8; i++)
			branchAngle[i] = (1.0-animationTime)*branchAngleHexapodView[i] + animationTime*branchAngleModuleView[i];
		graphAttribute.zeroRadius = animationTime*nodeDrawAttribute.masterRadius;
		graphAttribute.moduleRetract = animationTime*5*nodeDrawAttribute.nodeRadius;
		graphAttribute.posY = - animationTime*height/4;
	}
	
	
	
	
	/*
		Managment of the aside board
	*/
	if(graphAttribute.asideState == 0)
	{
		if(selectedNode >= 0 && selectedNode != 258 && selectedNode != 259)
		{
			graphAttribute.asideState = 1;
			animationAsideTime = 0.0;
		}
	}
	else if(graphAttribute.asideState == 1)
	{
		animationAsideTime += animationStep;
		if(animationAsideTime >= 1.0)
		{
			animationAsideTime = 1.0;
			graphAttribute.asideState = 2;
		}
		graphAttribute.posX = - animationAsideTime*width/5;
	}
	else if(graphAttribute.asideState == 2)
	{
		if(selectedNode < 0 || selectedNode == 258 || selectedNode == 259)
		{
			graphAttribute.asideState = 3;
			animationAsideTime = 1.0;
		}
	}
	else if(graphAttribute.asideState == 3)
	{
		animationAsideTime -= animationStep;
		if(animationAsideTime <= 0.0)
		{
			animationAsideTime = 0.0;
			graphAttribute.asideState = 0;
		}
		graphAttribute.posX = - animationAsideTime*width/5;
	}
}




function draw()
{
	clear();
	background(200);
	
	
	
	/*loopCounter++;
	if(loopCounter > 15)
	{
		loopCounter = 0;
		selectedNode++;
		if(selectedNode > 257)
		{
			selectedNode = 0;
		}
		else if(selectedNode == 26)
		{
			selectedNode = 256;
		}
	}*/
	
	updateGraphView();
	drawAllNets();
	drawAllNodes();
}

function mousePressed()
{
	if(mouseButton == LEFT)
		selectedNode = hoverNode;
}



