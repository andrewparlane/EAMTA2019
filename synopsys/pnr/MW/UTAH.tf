
Technology	{
		name				= ""
		date				= "Mar 26 2008"
		dielectric			= 3.45e-05
		unitTimeName			= "ns"
		timePrecision			= 1000
		unitLengthName			= "micron"
		lengthPrecision			= 1000
		gridResolution			= 150
		unitVoltageName			= "V"
		voltagePrecision		= 1000000
		unitCurrentName			= "uA"
		currentPrecision		= 1000000
		unitPowerName			= "mw"
		powerPrecision			= 1000000
		unitResistanceName		= "kohm"
		resistancePrecision		= 1000000
		unitCapacitanceName		= "pf"
		capacitancePrecision		= 1000000
		unitInductanceName		= "nh"
		inductancePrecision		= 100
		minBaselineTemperature		= 25
		nomBaselineTemperature		= 25
		maxBaselineTemperature		= 25
}

Color		27 {
		name				= "27"
		rgbDefined			= 1
		redIntensity			= 90
		greenIntensity			= 175
		blueIntensity			= 255
}

Color		43 {
		name				= "43"
		rgbDefined			= 1
		redIntensity			= 180
		greenIntensity			= 175
		blueIntensity			= 255
}

Tile		"unit" {
		width				= 2.4
		/* height				= 27 */
		height				= 30 
}

Layer		"poly" {
		layerNumber			= 46
		maskName			= "poly"
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "white"
		lineStyle			= "solid"
		pattern				= "blank"
		pitch				= 1.5
		defaultWidth		= 0.6
		minWidth			= 0.6
		minSpacing			= 0.9
}

Layer		"cc" {
		layerNumber			= 25
		maskName			= "polyCont"
		isDefaultLayer			= 1
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "27"
		lineStyle			= "solid"
		pattern				= "solid"
		pitch				= 0
		defaultWidth			= 0
		minWidth			= 0
		minSpacing			= 0.9
}

Layer		"metal1" {
		layerNumber			= 49
		maskName			= "metal1"
		isDefaultLayer			= 1
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "cyan"
		lineStyle			= "solid"
		pattern				= "dot"
		pitch				= 3
		defaultWidth			= 0.9
		minWidth			= 0.9
		minSpacing			= 0.9
		sameNetMinSpacing		= 0.9
		unitMinResistance		= 9e-05
		unitNomResistance		= 9e-05
		unitMaxResistance		= 9e-05
		unitMinCapacitance		= 0.000199
		unitNomCapacitance		= 0.000199
		unitMaxCapacitance		= 0.000199
		unitMinHeightFromSub		= 0.38
		unitNomHeightFromSub		= 0.38
		unitMaxHeightFromSub		= 0.38
		unitMinThickness		= 0.64
		unitNomThickness		= 0.64
		unitMaxThickness		= 0.64
}

Layer		"via" {
		layerNumber			= 50
		maskName			= "via1"
		isDefaultLayer			= 1
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "43"
		lineStyle			= "solid"
		pattern				= "rectangleX"
		pitch				= 0
		defaultWidth			= 0.6
		minWidth			= 0.6
		minSpacing			= 0.9
		fatTblDimension			= 2
		fatTblThreshold			= (1.2,120.001)
		fatTblThreshold2		= (1.2,120.001)
		fat2DTblFatContactNumber	= (5,1,
						               1,1)
		fat2DTblFatContactMinCuts	= (1,1,
						   1,1)
}

Layer		"metal2" {
		layerNumber			= 51
		maskName			= "metal2"
		isDefaultLayer			= 1
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "yellow"
		lineStyle			= "solid"
		pattern				= "dot"
		pitch				= 2.4
		defaultWidth			= 0.9
		minWidth			= 0.9
		minSpacing			= 0.9
		sameNetMinSpacing		= 0.9
		unitMinResistance		= 9e-05
		unitNomResistance		= 9e-05
		unitMaxResistance		= 9e-05
		unitMinCapacitance		= 0.000149
		unitNomCapacitance		= 0.000149
		unitMaxCapacitance		= 0.000149
		unitMinHeightFromSub		= 1.62
		unitNomHeightFromSub		= 1.62
		unitMaxHeightFromSub		= 1.62
		unitMinThickness		= 0.57
		unitNomThickness		= 0.57
		unitMaxThickness		= 0.57
}

Layer		"via2" {
		layerNumber			= 61
		maskName			= "via2"
		isDefaultLayer			= 1
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "blue"
		lineStyle			= "solid"
		pattern				= "blank"
		pitch				= 0
		defaultWidth			= 0.6
		minWidth			= 0.6
		minSpacing			= 0.9
		fatTblDimension			= 2
		fatTblThreshold			= (1.2,120.001)
		fatTblThreshold2		= (1.8,180.001)
		fat2DTblFatContactNumber	= (6,2,
						   2,2)
		fat2DTblFatContactMinCuts	= (1,1,
						   1,1)
}

Layer		"metal3" {
		layerNumber			= 62
		maskName			= "metal3"
		isDefaultLayer			= 1
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "red"
		lineStyle			= "solid"
		pattern				= "wave"
		pitch				= 3
		defaultWidth			= 1.5
		minWidth			= 1.5
		minSpacing			= 0.9
		sameNetMinSpacing		= 0.9
		unitMinResistance		= 5e-05
		unitNomResistance		= 5e-05
		unitMaxResistance		= 5e-05
		unitMinCapacitance		= 6.3e-05
		unitNomCapacitance		= 6.3e-05
		unitMaxCapacitance		= 6.3e-05
		unitMinHeightFromSub		= 2.52
		unitNomHeightFromSub		= 2.52
		unitMaxHeightFromSub		= 2.52
		unitMinThickness		= 0.77
		unitNomThickness		= 0.77
		unitMaxThickness		= 0.77
}

Layer		"OVERLAP" {
		layerNumber			= 8
		maskName			= ""
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "white"
		lineStyle			= "solid"
		pattern				= "blank"
		pitch				= 0
		defaultWidth			= 0
		minWidth			= 0
		minSpacing			= 0
}

ContactCode	"M2_M1_via" {
		contactCodeNumber		= 1
		cutLayer			= "via"
		lowerLayer			= "metal1"
		upperLayer			= "metal2"
		isDefaultContact		= 1
		cutWidth			= 0.6
		cutHeight			= 0.6
		upperLayerEncWidth		= 0.3
		upperLayerEncHeight		= 0.3
		lowerLayerEncWidth		= 0.3
		lowerLayerEncHeight		= 0.3
		minCutSpacing			= 0.9
}

ContactCode	"M3_M2_via" {
		contactCodeNumber		= 2
		cutLayer			= "via2"
		lowerLayer			= "metal2"
		upperLayer			= "metal3"
		isDefaultContact		= 1
		cutWidth			= 0.6
		cutHeight			= 0.6
		upperLayerEncWidth		= 0.6
		upperLayerEncHeight		= 0.6
		lowerLayerEncWidth		= 0.3
		lowerLayerEncHeight		= 0.3
		minCutSpacing			= 0.9
}

ContactCode	"M3_M2" {
		contactCodeNumber		= 3
		cutLayer			= "via2"
		lowerLayer			= "metal2"
		upperLayer			= "metal3"
		contactSourceType		= 5
		isFatContact			= 1
		cutWidth			= 0.6
		cutHeight			= 0.6
		upperLayerEncWidth		= 0.6
		upperLayerEncHeight		= 0.6
		lowerLayerEncWidth		= 0.3
		lowerLayerEncHeight		= 0.3
		minCutSpacing			= 0.9
}

ContactCode	"M2_M1" {
		contactCodeNumber		= 4
		cutLayer			= "via"
		lowerLayer			= "metal1"
		upperLayer			= "metal2"
		contactSourceType		= 5
		isFatContact			= 1
		cutWidth			= 0.6
		cutHeight			= 0.6
		upperLayerEncWidth		= 0.3
		upperLayerEncHeight		= 0.3
		lowerLayerEncWidth		= 0.3
		lowerLayerEncHeight		= 0.3
		minCutSpacing			= 0.9
}

ContactCode	"viagen21" {
		contactCodeNumber		= 5
		cutLayer			= "via"
		lowerLayer			= "metal1"
		upperLayer			= "metal2"
		contactSourceType		= 5
		isFatContact			= 1
		cutWidth			= 0.6
		cutHeight			= 0.6
		upperLayerEncWidth		= 0.3
		upperLayerEncHeight		= 0.3
		lowerLayerEncWidth		= 0.3
		lowerLayerEncHeight		= 0.3
		minCutSpacing			= 0.9
}

ContactCode	"viagen32" {
		contactCodeNumber		= 6
		cutLayer			= "via2"
		lowerLayer			= "metal2"
		upperLayer			= "metal3"
		contactSourceType		= 5
		isFatContact			= 1
		cutWidth			= 0.6
		cutHeight			= 0.6
		upperLayerEncWidth		= 0.6
		upperLayerEncHeight		= 0.6
		lowerLayerEncWidth		= 0.6
		lowerLayerEncHeight		= 0.6
		minCutSpacing			= 1.5
}

DesignRule	{
		layer1				= "cc"
		layer2				= "via"
		minSpacing			= 0
		minEnclosure			= 0
		stackable			= 1
}

DesignRule	{
		layer1				= "via"
		layer2				= "via2"
		minSpacing			= 0
		minEnclosure			= 0
		stackable			= 1
}

/*
    PRRules
*/
PRRule {
    rowSpacingTopTop                = 0
    rowSpacingBotBot                = 0
    abuttableTopTop                 = 1
    abuttableBotBot                 = 1
}

/*
# rredlich:  Density Rules 
*/

/*DesignRule      {
                layer1                          = "poly"
                layer2                          = "cc"
                minSpacing                      = 0
                minEnclosure                    = 0
                stackable                       = 1
}*/


ContactCode	"RVC0" {
		contactCodeNumber		= 7
		cutLayer			= "cc"
		lowerLayer			= "poly"
		upperLayer			= "metal1"
		isDefaultContact		= 1
		cutWidth			= 0.5
		cutHeight			= 0.5
		upperLayerEncWidth		= 0.2
		upperLayerEncHeight		= 0.2
		lowerLayerEncWidth		= 0.2
		lowerLayerEncHeight		= 0.2
		minCutSpacing			= 0.6
/*		maxNumRowsNonTurning		= 6*/
}

DensityRule {
                layer =  "poly"
                windowSize =  500
                minDensity =  14
                maxDensity =  30
		maxGradientDensity = 10
}


DensityRule { 
		layer =  "metal1" 
		windowSize =  500 
		minDensity =  30 
		maxDensity =  80 
		maxGradientDensity = 10
} 

DensityRule {
                layer =  "metal2"
                windowSize =  500
                minDensity =  30
                maxDensity =  80
		maxGradientDensity = 10
}

DensityRule {
                layer =  "metal3"
                windowSize =  500
                minDensity =  30
                maxDensity =  80
		maxGradientDensity = 10
}


/* 
# ronaldv:  Additional Layers (CEL VIEW) 
*/

Layer		"NWELL" {
		layerNumber			= 42
		maskName			= "nwell"
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "SILVER"
		lineStyle			= "MLINE"
		pattern				= "SLASHW"
		pitch				= 3.6
		defaultWidth			= 3.6
		minWidth			= 3.6
		minSpacing			= 5.4
		sameNetMinSpacing		= 1.8
}
Layer		"ACTIVE" {
		layerNumber			= 43
		maskName			= ""
		visible				= 1
		selectable			= 1
		blink				= 0
		color				= "white"
		lineStyle			= "solid"
		pattern				= "DOTS1"
		pitch				= 0.9
		defaultWidth			= 0.9
		minWidth			= 0.9
		minSpacing			= 0.9
		sameNetMinSpacing		= 0.9
}
Layer		"PSELECT" {
		layerNumber			= 44
		maskName			= ""
		visible				= 1
		selectable			= 1
		blink				= 0
		lineStyle			= "solid"
		pattern				= "IMPDOT"
		pitch				= 0.6
		defaultWidth			= 0.6
		minWidth			= 0.6
		minSpacing			= 0.6
		sameNetMinSpacing		= 0.6
}

Layer		"NSELECT" {
		layerNumber			= 45
		maskName			= ""
		visible				= 1
		selectable			= 1
		blink				= 0
		lineStyle			= "solid"
		pattern				= "IMPDOT"
		pitch				= 0.6
		defaultWidth			= 0.6
		minWidth			= 0.6
		minSpacing			= 0.6
		sameNetMinSpacing		= 0.6
}
/* Styles */

Stipple		"SLASHW" {
		width			= 16
		height			= 16
		pattern			= (0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 
					   0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 
					   0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 
					   1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 
					   0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 
					   0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 
					   0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
					   0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 
					   0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 
					   0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 
					   1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 
					   0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 
					   0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 
					   0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0) 
}

Stipple		"DOTS1" {
		width			= 16
		height			= 16
		pattern			= (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0) 
}

Stipple		"IMPDOT" {
		width			= 16
		height			= 16
		pattern			= (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
					   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) 
}

LineStyle	"MLINE" {
		width			= 2
		height			= 1
		pattern			= (1, 1) 
}
