#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "programmer.h"

/* include our function definitions explicit to this example */
#include "plugin_p.h"

static int simulation_duration;
static int g_nZones;          //number of zones
static int g_nDetectors;      //number of detectors
static int g_nLinks;	      //number of links
static int **g_ReleaseRates;  //car release rates between zones
static int g_ODRecords = 0;   //the default release rate when it couldn't be read
static Bool g_Setup = FALSE;  //setup complete flag

static int ***zones_cars;     //car release rates between zones for each minute
static int *min_amounts;      //car release rates of zones for current minute 
static int *dec_amounts;      //remaining car release rates of zones for current minute 
static int **link_detector_mapping;  //mapping between links and detectors, e.g link 1 contains detectors 2, 5, 6, 7.
static int *link_detector_count;     //the number of detectors in the link, e.g link 1 contains 4 detectors

static double ***detector_occ;	//occupancy values that have been read from detectors throughout the simulation

static int min = 0;  //starting minute

//california
static int d = 5;    // time interval
static int n = 3;    // time interval
static float ***occdf;    //temporary calculated values to test thresholds
static float ***occrdf;
static float ***docctd;
static int ***california_detector;	  //detector: 0 for normal traffic, 1 for incident 
static float t1_california = (float)0.1;  //thresholds
static float t2_california = (float)0.1; 
static float t3_california = (float)0.1;

//minnesota
static float ***occ_delta;
static float ***occ_delta_d;
static float ***max_occ;
static int ***minnesota_detector;	  //detector: 0 for normal traffic, 1 for incident 
static float t1_minnesota = (float)0.1;	  //thresholds
static float tc_minnesota = (float)0.1;

//smd
static int ***smd_detector;		  //detector: 0 for normal traffic, 1 for incident 
static float ts_smd = (float)0.1;	  //thresholds


/* Start control function calls */
/* ---------------------------------------------------------------------
 * call qpx_NET_postOpen once when the full network has been read into modeller
 * --------------------------------------------------------------------- */
void qpx_NET_postOpen(void) 
{ 
  int max_zones = qpg_NET_zones() * qpg_NET_zones();
  int i, j, k, detector_cnt, link_index;
	FILE *in;

	simulation_duration = qpg_CFG_duration() / 60;
  //multiple vehicle release at one time stamp is disabled
  qps_ZNE_vehicleRecall(FALSE);

  //time stamp is set to half second
  qps_CFG_timeStep(0.5);
	
  //the numbers of detectors and links are set
  g_nDetectors = qpg_NET_detectors();
  g_nLinks = qpg_NET_links();

  //allocate memory for mapping between links and detectors
  link_detector_mapping = malloc((g_nLinks) * sizeof(int *));
  if( link_detector_mapping == NULL)
  {
  	qps_GUI_printf("\nERROR: Cannot allocate memory for link_detector_mapping!\n");
    //exit(1);
  }
	
  //allocate memory for detector counts of links
  link_detector_count = malloc((g_nLinks) * sizeof(int));
  if( link_detector_count == NULL)
  {
  	qps_GUI_printf("\nERROR: Cannot allocate memory for link_detector_count!\n");
    //exit(1);
  }

  //initialize detectors counts of links and link-detector mapping
  for (j = 0; j < (g_nLinks); j++) {
  	detector_cnt = qpg_LNK_detectors(qpg_NET_linkByIndex(j+1));
  	link_detector_count[j] = detector_cnt;
  	link_detector_mapping[j] = malloc((detector_cnt) * sizeof(int));
  	if( link_detector_mapping[j] == NULL)
  	{
  		qps_GUI_printf("\nERROR: Cannot allocate memory for link_detector_mapping at index [%d]!\n",j);
  		//exit(1);
  	}
  	k = 0;
  	for (i = 0; i < (g_nDetectors); i++) {
  		link_index = qpg_LNK_index(qpg_DTC_link(qpg_NET_detectorByIndex(i+1)));
  		if(link_index == j+1)		
  		{
  			link_detector_mapping[j][k] = i;
  			k++;
  		}
  	}
  }

  //allocate memory for 3d array of occupancy values
  //1-link index
  //2-detector index
  //3-minute occupancy
  detector_occ = malloc((g_nLinks) * sizeof(int *));
  if( detector_occ == NULL)
  {
  	qps_GUI_printf("\nERROR: Cannot allocate memory for detector_occ!\n");
    //exit(1);
  }
  for (i = 0; i < (g_nLinks); i++) {
  	detector_occ[i] = malloc(link_detector_count[i] * sizeof(int *));
    if( detector_occ[i] == NULL)
    {
      qps_GUI_printf("\nERROR: Cannot allocate memory for detector_occ at index [%d]!\n",i);
	    //exit(1);
    }
    for(j = 0; j < (link_detector_count[i]); j++) {
      detector_occ[i][j] = malloc(simulation_duration * sizeof(double));
      if( detector_occ[i][j] == NULL)
      {
		    qps_GUI_printf("\nERROR: Cannot allocate memory for detector_occ at index [%d][%d]!\n",i,j);
		    //exit(1);
      }
    }
  }

  //initialize allocated memory
  for (i = 0; i < (g_nLinks); i++) {
    for (j = 0; j < (link_detector_count[i]); j++) {
      for(k = 0; k < (simulation_duration); k++) {
        detector_occ[i][j][k] = 0;
	    }
    }
  }

  //allocate memory for release rates between zones for each minute
  zones_cars = malloc((g_nZones) * sizeof(int *));
  if( zones_cars == NULL)
  {
    qps_GUI_printf("\nERROR: Cannot allocate memory for zones_cars!\n");
    //exit(1);
  }
  for (i = 0; i < (g_nZones); i++) {
  	zones_cars[i] = malloc((g_nZones) * sizeof(int *));
    if( zones_cars[i] == NULL)
    {
		  qps_GUI_printf("\nERROR: Cannot allocate memory for zones_cars at index [%d]!\n",i);
		  //exit(1);
    }
    for(j = 0; j < (g_nZones); j++) {
      zones_cars[i][j] = malloc(simulation_duration * sizeof(int));
	    if( zones_cars[i][j] == NULL)
	    {
        qps_GUI_printf("\nERROR: Cannot allocate memory for zones_cars at index [%d][%d]!\n",i,j);
        //exit(1);
	    }
    }
  }

  //allocate memory for car release rates of zones for current minute 
  min_amounts = malloc((g_nZones) * sizeof(int));
  if( min_amounts == NULL)
  {
  	qps_GUI_printf("\nERROR: Cannot allocate memory for min_amounts!\n");
    //exit(1);
  }

  //allocate memory for remaining car release rates of zones for current minute
  dec_amounts = malloc((g_nZones) * sizeof(int));
  if( dec_amounts == NULL)
  {
  	qps_GUI_printf("\nERROR: Cannot allocate memory for dec_amounts!\n");
  	//exit(1);
  }

  //read release rates between zones for each minute
  //release rates must be in the same directory with .dll and named to "input.txt"
  //Format for n zones and 1 hour simulation is [(n-1)*n X simulation_duration] matrix
  // i.e n = 3
  //1. zone to 2. zone simulation_duration rate 
  //1. zone to 3. zone simulation_duration rate
  //2. zone to 1. zone simulation_duration rate
  //2. zone to 3. zone simulation_duration rate
  //3. zone to 1. zone simulation_duration rate
  //3. zone to 2. zone simulation_duration rate
  // in = fopen("input.txt", "r");
  // if (in == NULL) {
	//   qps_GUI_printf("\nERROR: Cannot open input.txt file \n");
	//   //exit(1);
  // }
  // for(i=0; i < g_nZones; i++) {
	//   for(j=0; j < g_nZones; j++) {
	//	   for(k=0; k < simulation_duration; k++) {
	//		   if(i==j) zones_cars[i][j][k] = 0;
	//		   else fscanf(in, "%d", &zones_cars[i][j][k]);
	//	   }
	//   }	
  // }
  // fclose(in);

  //setup is successfully completed!
  qps_GUI_printf("\nParamics Programmer API: Vehicle Release\n");
  g_Setup = TRUE;
} 

/* ----------------------------------------------------------------------- 
 * This call allows the plugin to setup any required structures needed for 
 * the API interface based on data provided in the parameters file.
 * ----------------------------------------------------------------------- */
void qpx_CFG_parameterFile(char *filename, int count) 
{
  if (strcmp(filename,g_ParamFile ) == 0)
  {
    /* estimate how many OD records the user has supplied  NOTE: all 
       coefficients supplied should be od records except the first */ 
    g_ODRecords = count - 1;
  }
  g_Setup = FALSE;
}

/* ----------------------------------------------------------------------- 
 * This call is made for each of the parameters supplied in the plugin
 * parameter file and is used to store each of the users variables. In
 * this example we will use it to store the trip rates for each of our OD 
 * pairs supplied in the parameters file. Note that these values will be 
 * updated if the user changes a parameter via the GUI slider bar.
 * ----------------------------------------------------------------------- */ 
void qpx_GUI_parameterValue(char *filename,int index,char *label,float value) 
{
  int from, to, max_release, i, j;   

  /* check he file name */
  if (strcmp(filename, g_ParamFile) == 0)
  {
  	/* here we must assume that the first coefficient in the API Parameters 
  	 * file is the number of zones in the network.  We need to do this in 
  	 * order to properly dimension our storage array before the trip 
  	 * records can be read and stored */

    if ((strcmp(label, "Zones in network") == 0) && (index == 0))
	  {
	    /* the first line of the file with the correct label to identify
		   * the number of zones in the network */
	    g_nZones = value;

      /* as we now know the size of the matrix we need to build we can
	     * allocate storage */
	    if (g_ReleaseRates != NULL) free(g_ReleaseRates);
		  g_ReleaseRates = malloc((g_nZones) * sizeof(int *));
		  if( g_ReleaseRates == NULL)
		  {
        qps_GUI_printf("\nERROR: Cannot allocate memory for g_ReleaseRates!\n");
			  //exit(1);
		  }
	    for (i = 0; i < (g_nZones); i++)
		  {
        g_ReleaseRates[i] = malloc((g_nZones) * sizeof(int));
        if( g_ReleaseRates[i] == NULL)
        {
				  qps_GUI_printf("\nERROR: Cannot allocate memory for g_ReleaseRates at index [%d]!\n",i);
				  //exit(1);
        }
		  }
   
	     /* now clear all elements of our matrix prior to reading in the
	      * values from our coefficient file */
	    for (i =0; i < (g_nZones); i++)
	    {
		    for(j = 0; j < (g_nZones); j++)
		      g_ReleaseRates[i][j] = 0;
	    }
    }
    else if ((strcmp(label, "Zones in network") == 0) && (index != 0))
    {
      /* the label identifying the number of zones in the network has
      * been found but it is not the first coefficient in the 
      * Parameters file :- there is a possibility that the user has 
      * placed the coefficients  in the wrong order, this will cause the 
      * plugin to fail so warn the user */
      qps_GUI_printf("\nWARNING: Possible error in parameters file coefficient \n"
      "         values ordering, please check you input data !");
      return;
	 }
	 else
	 {
      /* store the trip record

      /* scan the label name to find the OD pair */
      sscanf(label, "Trips Zone %d to Zone %d", &from, &to);

      /* check for sensible zone numbers in the range 1 - g_nZones */
      if (((from < 1) || (to < 1)) || ((from > g_nZones) || (to > g_nZones)))
      {
        qps_GUI_printf("\nWarning: Invalid OD pair - coefficient %d in parameters "
		                   "file\n", index);
        return;
      }

      /* store trips to be released for the correct OD pair */
      g_ReleaseRates[from-1][to-1] = value;
    } 
  }
}

/* ----------------------------------------------------------------------- 
 * This function sets all the variables needed for releasing a vehicle into
 * our network.
 * ----------------------------------------------------------------------- */
static void pp_release_vehicle(int type, int dest)
{
  /* normal will be used as the distrution for our DVU's aggression and 
   * awarness factors */
  int normal[9] =  {1, 4, 11, 21, 26, 21, 11, 4, 1};
  int aggr, awar, sum, new_sum, i;
  /* maximum integer size */
  int max_rand = 32767;
    
  /* this callback function set the type of vehicle to be released from the 
   * zone */
  qps_ZNE_vehicleType(type);
  /* this callback function set the destination zone index of the vehicle
   * about to be released */
  qps_ZNE_vehicleDestination(dest);
    
    
  /* calculate aggression and awareness factors */
  aggr = (((float)rand()/(float)(max_rand)) *100.0);
  awar = (((float)rand()/(float)(max_rand)) *100.0);

  sum = 0;
  for(i = 0; i < 9; i++)
  {
    new_sum = sum + normal[i];       
    if((aggr > sum) && (aggr <= new_sum))
    {
      qps_ZNE_vehicleAggression(i);
    }

    if((awar > sum) && (awar <= new_sum))
    {
      qps_ZNE_vehicleAwareness(i);
    }       
    sum = new_sum;
  }
}

/* ----------------------------------------------------------------------- 
 * This control function is called for each zone in the network for each 
 * time step, to determine if a vehicle should be released or not. 
 * ----------------------------------------------------------------------- */

//VEHICLE RELEASE SECTION STARTS

//void qpx_ZNE_timeStep(ZONE* zone) 
//{
//    //
//    //rand is old, where it is used
//    //
//    double r = rand() / (double)32767;    //get a random number
//    int from_zone;                        //zone number that the vehicle will be released
//    int i, j, k, sum, amount;             //temporary index and variables
//    int minutes = floor(qpg_CFG_simulationTime() / 60);  //current minute
//
//    //informative
//    //qps_GUI_printf("timestep: %f \n", qpg_CFG_timeStep());
//    //qps_GUI_printf("zones: %d \n", g_nZones);
//    //qps_GUI_printf("index: %d \n", qpg_ZNE_index(zone));
//    //qps_GUI_printf("time: %f \n", qpg_CFG_simulationTime());
//
//    //get the zone number 
//    from_zone = qpg_ZNE_index(zone) - 1;
//
//    //get the release rates of the current minute
//    //this function will be called just one time for whole network at each minute
//    if(from_zone == 0 && (int)(qpg_CFG_simulationTime() - 0.5) % 60 == 0) {
//
//	for(i = 0; i < g_nZones; i++) {
//		
//	    for(j = 0; j < g_nZones; j++) {
//	
//		g_ReleaseRates[i][j] = zones_cars[i][j][minutes];
//		//qps_GUI_printf("%d\t", g_ReleaseRates[i][j]);
//
//      	    }
//
//	    qps_GUI_printf("\n");			
//
//	}
//	
//	for(i = 0; i < g_nZones; i++) {
//	    sum = 0;
//	    for(j = 0; j < g_nZones; j++) {
//	
//		sum += g_ReleaseRates[i][j];
//			
//	    }
//		
//	    min_amounts[i] = sum;
//	    dec_amounts[i] = sum;
//	}
//   }
//	/*
//	if(from_zone == 2) {
//		if(r < 0.5) {
//			qps_GUI_printf("3 nolu zone 1 nolu zone a araba gonderdim \n");
//			pp_release_vehicle(from_zone + 1, 1);
//		} else {
//			qps_GUI_printf("3 nolu zone 2 nolu zone a araba gonderdim \n");
//			pp_release_vehicle(from_zone + 1, 2);
//		} 
//	}else if(from_zone == 1) {
//		if(r < 0.5) {
//			qps_GUI_printf("2 nolu zone 1 nolu zone a araba gonderdim \n");
//			pp_release_vehicle(from_zone + 1, 1);
//		} else {
//			qps_GUI_printf("2 nolu zone 3 nolu zone a araba gonderdim \n");
//			pp_release_vehicle(from_zone + 1, 3);
//		} 
//	} else {
//		if(r < 0.5) {
//			qps_GUI_printf("1 nolu zone 2 nolu zone a araba gonderdim \n");
//			pp_release_vehicle(from_zone + 1, 2);
//		} else {
//			qps_GUI_printf("1 nolu zone 3 nolu zone a araba gonderdim \n");
//			pp_release_vehicle(from_zone + 1, 3);
//		} 
//	}
//	*/
//
//    //this is main function to release vehicle that is called for each zone at each timestep
//    //checks whether this zone will send vehicles to other zones
//    //then sends vehicles by adjusting rate
//    if(dec_amounts[from_zone] > 0) {
//	amount = 120 / min_amounts[from_zone];
//	if((((int)(qpg_CFG_simulationTime() * 2) % 120) % amount == 0) && dec_amounts[from_zone] > 0) {
//	
//	   for(i=0; i<(g_nZones); i++) {
//		
//		if(g_ReleaseRates[from_zone][i] > 0) {
//			
//			pp_release_vehicle(from_zone + 1, i + 1);	
//			dec_amounts[from_zone]--;
//			g_ReleaseRates[from_zone][i]--;
//			break;
//			
//		}	
//
//           }
//	
//	}
//    }
//
//}

//VEHICLE RELEASE SECTION ENDS

//this function is called when a vehicle passes a detector
void qpx_VHC_detector(VEHICLE* vehicle, LINK* link, DETECTOR* detector) {

  //temporary indexes
	int i, j, index;     
  //the number of the detector that detected a vehicle
	int detector_index = qpg_DTC_index(detector) - 1;   
  //the number of the link that detector belongs to
	int link_index = qpg_LNK_index(link) - 1;   

  //current minute of the simulation
	int lmin = floor(qpg_NET_wholeSeconds()/(double)60);   
	float occ;  //occupancy that has been read

  //find the index of the detector in link-to-detector mapping
	for (j = 0; j < (link_detector_count[link_index]); j++) {
		if(detector_index == link_detector_mapping[link_index][j]) {
			index = j;
			break;	
		}
	}

  //if we are in the same minute, not start new minute
	if(lmin == min) {
    //sum all the lanes for the detector
		for(i=qpg_DTC_lane(detector); i<=qpg_DTC_end_lane(detector); i++){
			occ = qpg_DTL_occupancy(qpg_DTC_multipleLoop(detector, i), APILOOP_SMOOTHED);

			//correct read occupancy if it is negative or higher than 1
			if(occ < 0) occ = 0;
			else if(occ > 1) occ = 1;

			detector_occ[link_index][index][lmin] += occ;		
			//qps_GUI_printf("%d %f", detector_index, occ);
		}
	} else {
		// we started a new minute 
		// so take the average of the occupancies
		for(j=0; j<g_nLinks; j++) {
			for(i=0; i<(link_detector_count[j]); i++) {
        //for 1 line minute occ 3*simulation_duration
				detector_occ[j][i][min] /= (3*simulation_duration/qpg_CFG_timeStep()); 
			}
		}
	 	
		//advance the global simulation time
		min++;
                
    //read the occupancy as in first 'if' part
		for(i=qpg_DTC_lane(detector); i<=qpg_DTC_end_lane(detector); i++){
  		occ = qpg_DTL_occupancy(qpg_DTC_multipleLoop(detector, i), APILOOP_SMOOTHED);

			if(occ < 0) occ = 0;
			else if(occ > 1) occ = 1;

			detector_occ[link_index][index][lmin] += occ;
		}
	}
}

//this function is called when a hour is passed, 
//actually simulation finished because our simulation will be "simulation_duration" minutes
void qpx_NET_complete() {

	int i, j, k, l;    //temporary indexes
  //mean and stardard deviation of arrays that are required for calculations of algorithms 
	float mean, std;   
	FILE *file;        

	//allocate memory for occdf of california
	occdf = malloc((g_nLinks) * sizeof(float *));
	if( occdf == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for occdf!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		occdf[i] = malloc(link_detector_count[i] * sizeof(float *));
		if( occdf[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for occdf at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			occdf[i][j] = malloc(simulation_duration * sizeof(float));
			if( occdf[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for occdf at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}

	//allocate memory for occrdf of california
	occrdf = malloc((g_nLinks) * sizeof(float *));
	if( occrdf == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for occrdf!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		occrdf[i] = malloc(link_detector_count[i] * sizeof(float *));
		if( occrdf[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for occrdf at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			occrdf[i][j] = malloc(simulation_duration * sizeof(float));
			if( occrdf[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for occrdf at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}

	//allocate memory for docctd of california
	docctd = malloc((g_nLinks) * sizeof(float *));
	if( docctd == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for docctd!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		docctd[i] = malloc(link_detector_count[i] * sizeof(float *));
		if( docctd[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for docctd at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			docctd[i][j] = malloc(simulation_duration * sizeof(float));
			if( docctd[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for docctd at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}
	
	//allocate memory for california detector
	california_detector = malloc((g_nLinks) * sizeof(int *));
	if( california_detector == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for california_detector!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		california_detector[i] = malloc(link_detector_count[i] * sizeof(int *));
		if( california_detector[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for california_detector at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			california_detector[i][j] = malloc(simulation_duration * sizeof(int));
			if( california_detector[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for california_detector at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}

	//allocate memory for delta occ of minnesota
	occ_delta = malloc((g_nLinks) * sizeof(float *));
	if( occ_delta == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for occ_delta!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		occ_delta[i] = malloc(link_detector_count[i] * sizeof(float *));
		if( occ_delta[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for occ_delta at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			occ_delta[i][j] = malloc(simulation_duration * sizeof(float));
			if( occ_delta[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for occ_delta at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}

	//allocate memory for delta occ with d interval of minnesota
	occ_delta_d = malloc((g_nLinks) * sizeof(float *));
	if( occ_delta_d == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for occ_delta_d!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		occ_delta_d[i] = malloc(link_detector_count[i] * sizeof(float *));
		if( occ_delta_d[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for occ_delta_d at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			occ_delta_d[i][j] = malloc(simulation_duration * sizeof(float));
			if( occ_delta_d[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for occ_delta_d at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}

	//allocate memory for max occ of minnesota
	max_occ = malloc((g_nLinks) * sizeof(float *));
	if( max_occ == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for max_occ!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		max_occ[i] = malloc(link_detector_count[i] * sizeof(float *));
		if( max_occ[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for max_occ at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			max_occ[i][j] = malloc(simulation_duration * sizeof(float));
			if( max_occ[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for max_occ at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}
	
	//allocate memory for minnesota detector
	minnesota_detector = malloc((g_nLinks) * sizeof(int *));
	if( minnesota_detector == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for minnesota_detector!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		minnesota_detector[i] = malloc(link_detector_count[i] * sizeof(int *));
		if( minnesota_detector[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for minnesota_detector at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			minnesota_detector[i][j] = malloc(simulation_duration * sizeof(int));
			if( minnesota_detector[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for minnesota_detector at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}

	//allocate memory for smd detector
	smd_detector = malloc((g_nLinks) * sizeof(int *));
	if( smd_detector == NULL)
	{
		qps_GUI_printf("\nERROR: Cannot allocate memory for smd_detector!\n");
		//exit(1);
	}
	for (i = 0; i < (g_nLinks); i++) {
		smd_detector[i] = malloc(link_detector_count[i] * sizeof(int *));
		if( smd_detector[i] == NULL)
		{
			qps_GUI_printf("\nERROR: Cannot allocate memory for smd_detector at index [%d]!\n",i);
			//exit(1);
		}
		for(j = 0; j < (link_detector_count[i]); j++) {
			smd_detector[i][j] = malloc(simulation_duration * sizeof(int));
			if( smd_detector[i][j] == NULL)
			{
				qps_GUI_printf("\nERROR: Cannot allocate memory for smd_detector at index [%d][%d]!\n",i,j);
				//exit(1);
			}
		}
	}

	//at each second we add up occupacies and at each new minute
   	//we take average of old minute in 'timestep' function
   	//but when simulation is completed, last average couldn't be calculated in timestep
	//so here we take the average for the last minute 
	for(j=0; j<g_nLinks; j++) {
		for(i=0; i<(link_detector_count[j]); i++) {
			detector_occ[j][i][simulation_duration-1] /= (3*simulation_duration/qpg_CFG_timeStep()); //for 1 line minute occ 3*simulation_duration
		}
	}
	
	//california calculations
	for(i=0; i < g_nLinks; i++) {
		for(j=0; j < (link_detector_count[i]); j++) {
			for(k=0; k < simulation_duration; k++) {
				if(j != (link_detector_count[i]) - 1) {
					occdf[i][j][k] = detector_occ[i][j][k] - detector_occ[i][j+1][k];

					if(detector_occ[i][j][k] != 0)
						occrdf[i][j][k] = (detector_occ[i][j][k] - detector_occ[i][j+1][k]) / detector_occ[i][j][k];
					else 
						occrdf[i][j][k] = 0;

					if(k >= d)
						if(detector_occ[i][j+1][k-d] != 0)
							docctd[i][j][k] = (detector_occ[i][j+1][k-d] - detector_occ[i][j+1][k]) / detector_occ[i][j+1][k-d];
						else 
							docctd[i][j][k] = 0;
					else 
						docctd[i][j][k] = 0;

				} else {
					occdf[i][j][k] = detector_occ[i][j][k];

					if(detector_occ[i][j][k] != 0)
						occrdf[i][j][k] = detector_occ[i][j][k] / detector_occ[i][j][k];
					else 
						occrdf[i][j][k] = 0;

					docctd[i][j][k] = 0;
				}

				if(occdf[i][j][k] > t1_california && occrdf[i][j][k] > t2_california && docctd[i][j][k] > t3_california) california_detector[i][j][k] = 1;
				else california_detector[i][j][k] = 0;
			}
		}	
	}

	//minnesota calculations
	for(i=0; i < g_nLinks; i++) {
		for(j=0; j < (link_detector_count[i]); j++) {
			for(k=0; k < simulation_duration; k++) {
				if(k < simulation_duration-d)
				{
					if(j != (link_detector_count[i]) - 1) {
						occ_delta[i][j][k] = detector_occ[i][j][k+d] - detector_occ[i][j+1][k+d];

						if(k >= n) {
							occ_delta_d[i][j][k] = (detector_occ[i][j][k-n] - detector_occ[i][j+1][k-n]);
							max_occ[i][j][k] = max(detector_occ[i][j][k-n], detector_occ[i][j+1][k-n]);	
						} else {
							occ_delta_d[i][j][k] = 0;
							max_occ[i][j][k] = 0;
						}


					} else {
						occ_delta[i][j][k] = detector_occ[i][j][k+d];

						if(k >= d){
							occ_delta_d[i][j][k] = detector_occ[i][j][k-n];
							max_occ[i][j][k] = detector_occ[i][j][k-n];
						} else {
							occ_delta_d[i][j][k] = 0;
							max_occ[i][j][k] = 0;
						}
					}
					if(k >= n) {
						if(max_occ[i][j][k] != 0) {
							if(occ_delta[i][j][k]/max_occ[i][j][k] > t1_minnesota && (occ_delta[i][j][k]-occ_delta[i][j][k])/max_occ[i][j][k] > tc_minnesota) 
								minnesota_detector[i][j][k] = 1;
							else minnesota_detector[i][j][k] = 0;
						} else {
							minnesota_detector[i][j][k] = 0;
						}
					} else {
						if(max_occ[i][j][k] != 0) {
							if(occ_delta[i][j][k]/max_occ[i][j][k] > t1_minnesota && occ_delta[i][j][k]/max_occ[i][j][k] > tc_minnesota) 
									minnesota_detector[i][j][k] = 1;
							else minnesota_detector[i][j][k] = 0;
						} else {
							minnesota_detector[i][j][k] = 0;
						}
					}
				}
				else
				{
					occ_delta_d[i][j][k] = 0;
					max_occ[i][j][k] = 0;
					minnesota_detector[i][j][k] = 0;
				}

					
			}
		}	
	}

	file = fopen("\\path\\to\\ts_threshold.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open ts_threshold.txt file \n");
		//exit(1);
	}

	//smd calculations 
	for(i=0; i < g_nLinks; i++) {
		for(j=0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k=0; k < simulation_duration; k++) {
				mean = 0;
				std = 0;
				if(k >= d) {
					for(l=k-d; l<k; l++) mean += detector_occ[i][j][l];
					mean /= d;
					for(l=k-d; l<k; l++) std += (mean - detector_occ[i][j][l]) * (mean - detector_occ[i][j][l]);
					std /= (d-1);
					std = sqrtf(std);
				} else {
					for(l=0; l<k; l++) mean += detector_occ[i][j][l];
					mean /= k;
					for(l=k-d; l<k; l++) std += (mean - detector_occ[i][j][l]) * (mean - detector_occ[i][j][l]);
					std /= k;
					std = sqrtf(std);
				}

				if(std != 0) {
					if((detector_occ[i][j][k] - mean) / std >= ts_smd) smd_detector[i][j][k] = 1;
					else smd_detector[i][j][k] = 0;
					fprintf(file, "%f ", (detector_occ[i][j][k] - mean) / std);
				} else {
					smd_detector[i][j][k] = 0;
					fprintf(file, "0 ");
				}
			}
			fprintf(file, "\n");
		}	
		fprintf(file, "\n");
	}
	fclose(file);
	
	
	//For further analysis 
  //we have written each variable of the algorithms into files
	file = fopen("\\path\\to\\detector_occ.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open detector_occ.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%f ", detector_occ[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\occdf.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open occdf.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%f ", occdf[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\occrdf.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open occrdf.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%f ", occrdf[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\docctd.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open docctd.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%f ", docctd[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\california_detector.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open california_detector.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%d ", california_detector[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\occ_delta.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open occ_delta.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%f ", occ_delta[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\occ_delta_d.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open occ_delta_d.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%f ", occ_delta_d[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\max_occ.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open max_occ.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%f ", max_occ[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\minnesota_detector.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open minnesota_detector.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%d ", minnesota_detector[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	fclose(file);
	
	file = fopen("\\path\\to\\smd_detector.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open smd_detector.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				fprintf(file, "%d ", smd_detector[i][j][k]);
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	
	fclose(file);
	
	// writing results to the file starts here

	file = fopen("\\path\\to\\tc_threshold.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open tc_threshold.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				if(max_occ[i][j][k] != 0)
					fprintf(file, "%f ", occ_delta[i][j][k] / max_occ[i][j][k]);
				else
					fprintf(file, "0 ");
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	
	fclose(file);

	// writing results to the file ends here

	file = fopen("\\path\\to\\ti_threshold.txt", "w");
	if (file == NULL) {
		qps_GUI_printf("\nERROR: Cannot open ti_threshold.txt file \n");
		//exit(1);
	}
	
	for (i = 0; i < (g_nLinks); i++) {
		for (j = 0; j < (link_detector_count[i]); j++) {
			fprintf(file, "LINK #%d DETECTOR #%d ",i,j);
			for(k = 0; k < (simulation_duration); k++) {
				if(k>=d)
				{
					if(max_occ[i][j][k] != 0)
						fprintf(file, "%f ", (occ_delta[i][j][k] - occ_delta[i][j][k-d]) / max_occ[i][j][k]);
					else
						fprintf(file, "0 ");
				}
				else
				{
					if(max_occ[i][j][k] != 0)
						fprintf(file, "%f ", occ_delta[i][j][k] / max_occ[i][j][k]);
					else
						fprintf(file, "0 ");
				}
			}
			fprintf(file, "\n");
		}
		fprintf(file, "\n");
	}
	
	fclose(file);

	//deallocation of the allocated memory
	for(i=0; i < g_nLinks; i++) {
		for(j=0; j < (link_detector_count[i]); j++) {
				free(detector_occ[i][j]);
				free(occdf[i][j]);
				free(occrdf[i][j]);
				free(docctd[i][j]);
				free(california_detector[i][j]);
				free(occ_delta[i][j]);
				free(occ_delta_d[i][j]);
				free(max_occ[i][j]);
				free(minnesota_detector[i][j]);
				free(smd_detector[i][j]);
		}	
		free(detector_occ[i]);
		free(occdf[i]);
		free(occrdf[i]);
		free(docctd[i]);
		free(california_detector[i]);
		free(occ_delta[i]);
		free(occ_delta_d[i]);
		free(max_occ[i]);
		free(minnesota_detector[i]);
		free(smd_detector[i]);
	}
	free(detector_occ);
	free(occdf);
	free(occrdf);
	free(docctd);
	free(california_detector);
	free(occ_delta);
	free(occ_delta_d);
	free(max_occ);
	free(minnesota_detector);
	free(smd_detector);
	
	for(i=0; i < g_nZones; i++) {
		free(zones_cars[i]);
		free(g_ReleaseRates[i]);
	}
	free(zones_cars);
	free(g_ReleaseRates);
	
	for(i=0; i < g_nLinks; i++) {
		free(link_detector_mapping[i]);
	}
	free(link_detector_mapping);
	
	free(min_amounts);
	free(dec_amounts);
	free(link_detector_count);
}