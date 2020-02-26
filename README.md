# FTA_TransitSystems
Theses are codes for the simulation of fixed route, flexible route, and door-to-door service. Some parameters are originated from MTA B63 Bus route, connecting Cobble Hill and Bay Ridge, as an example.
## [Description of files]
### == MAIN CODES ==
#### 1. Main.m
  : functioning as a control panel of inputs, and users can adjust simulation parameters and choose a system to simulation.
#### 2. FixedRoute.m / FlexibleRoute.m / DtD.m
  : representing each system type (fixed route system / flexible route system / door-to-door service) and identifying simulation elements, initialize vehicle information, determine passenger-vehicle matches, and produce simulation outputs by organizing other function accordingly.


### == COMMON FUNCTIONS (with appropriate suffixes) ==
#### 3. IdentPax.m (_Fix,_Flex,_DtD)
  : receiving passenger information and prepare arrays for intermediate outputs
#### 4. Insert.m (_Fix,_Flex,_DtD)
  : conducting insertion heuristic to evaluate candidate routes and archive the best one for each vehicle
#### 5. Dwell.m (_Fix,_Flex,_DtD)
  : processing passenger pickup when staying at stops and determine vehicles’ movement when leaving stops
#### 6. VehBrdAlght.m (_Fix,_Flex,_DtD)
  : determining vehicles’ movement when moving and process passenger pickup and drop-off when arriving at stops

### == SPECIFIC FUNCTIONS FOR FLEXIBLE ROUTE SYSTEM ==
#### 7. PaxApproach.m
  : identifying feasible segments for passengers’ approach
#### 8. EvalSeg.m
  : evaluating candidate routes considering intersections with approaching passengers and archive the best one for each vehicle

### == TECHNICAL FUNCTIONS ==
#### 9. GridDist.m
  : receiving passenger information and prepare arrays for intermediate outputs
#### 10. Navigate.m
  : yielding the anticipated location after a given time or calculate a required time to reach the next point
  
### == ETC ==
#### 11. PaxGen.m
  : generating artificial passengers within the service area according to the demand level if data should be generated
  
### == SAMPLE DATASET ==
#### 12. Pool_N_Random.mat (80/200/400)
