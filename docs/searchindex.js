Search.setIndex({docnames:["[0].overview","[1].simulator","[2].intersection","[3].signal","[4].trajectory","index"],envversion:53,filenames:["[0].overview.rst","[1].simulator.rst","[2].intersection.rst","[3].signal.rst","[4].trajectory.rst","index.rst"],objects:{"":{main:[0,0,0,"-"]},"data.data":{get_conflict_dict:[1,1,1,""],get_general_params:[1,1,1,""],get_phases:[1,1,1,""],get_pretimed_parameters:[1,1,1,""],get_signal_params:[1,1,1,""]},"src.intersection":{Intersection:[2,2,1,""],Lanes:[2,2,1,""],Traffic:[2,2,1,""],Vehicle:[2,2,1,""]},"src.intersection.Intersection":{__init__:[2,3,1,""],get_det_range:[2,3,1,""],get_max_speed:[2,3,1,""],get_min_headway:[2,3,1,""],get_num_lanes:[2,3,1,""],get_poly_params:[2,3,1,""]},"src.intersection.Lanes":{__init__:[2,3,1,""],all_served:[2,3,1,""],decrement_first_unsrvd_indx:[2,3,1,""],decrement_last_veh_indx:[2,3,1,""],increment_first_unsrvd_indx:[2,3,1,""],increment_last_veh_indx:[2,3,1,""],purge_served_vehs:[2,3,1,""],reset_first_unsrvd_indx:[2,3,1,""]},"src.intersection.Traffic":{__init__:[2,3,1,""],close_trj_csv:[2,3,1,""],get_first_detection_time:[2,3,1,""],get_volumes:[2,4,1,""],last_veh_arrived:[2,3,1,""],save_csv:[2,3,1,""],serve_update_at_stop_bar:[2,3,1,""],set_departure_time_for_csv:[2,3,1,""],set_elapsed_sim_time:[2,3,1,""],update_vehicles_info:[2,3,1,""]},"src.intersection.Vehicle":{EPS:[2,5,1,""],MAX_NUM_TRAJECTORY_POINTS:[2,5,1,""],MIN_DIST_TO_STOP_BAR:[2,5,1,""],__init__:[2,3,1,""],check_trj_redo_needed:[2,3,1,""],map_veh_type2str:[2,4,1,""],print_trj_points:[2,3,1,""],reset_trj_points:[2,3,1,""],set_earliest_arrival:[2,3,1,""],set_first_trj_point_indx:[2,3,1,""],set_last_trj_point_indx:[2,3,1,""],set_poly_coeffs:[2,3,1,""],set_scheduled_arrival:[2,3,1,""]},"src.signal":{ActuatedControl:[3,2,1,""],Enumerate_SpaT:[3,2,1,""],GA_SPaT:[3,2,1,""],MinCostFlow_SPaT:[3,2,1,""],Pretimed:[3,2,1,""],Signal:[3,2,1,""]},"src.signal.ActuatedControl":{LAG:[3,5,1,""],LARGE_NUM:[3,5,1,""],__init__:[3,3,1,""],_flush_upcoming_SPaTs:[3,3,1,""],_schedule_unserved_vehicles:[3,3,1,""],_set_lane_lane_incidence:[3,3,1,""],_set_non_base_scheduled_arrival:[3,3,1,""],_set_phase_lane_incidence:[3,3,1,""],append_extend_phase:[3,3,1,""],base_badness:[3,3,1,""],set_critical_phase_volumes:[3,3,1,""],update_SPaT:[3,3,1,""]},"src.signal.Enumerate_SpaT":{LAG:[3,5,1,""],LARGE_NUM:[3,5,1,""],__init__:[3,3,1,""],_flush_upcoming_SPaTs:[3,3,1,""],_schedule_unserved_vehicles:[3,3,1,""],_set_lane_lane_incidence:[3,3,1,""],_set_non_base_scheduled_arrival:[3,3,1,""],_set_phase_lane_incidence:[3,3,1,""],append_extend_phase:[3,3,1,""],base_badness:[3,3,1,""],set_critical_phase_volumes:[3,3,1,""],solve:[3,3,1,""],update_SPaT:[3,3,1,""]},"src.signal.GA_SPaT":{BADNESS_ACCURACY:[3,5,1,""],CROSSOVER_SIZE:[3,5,1,""],LAG:[3,5,1,""],LAMBDA:[3,5,1,""],LARGE_NUM:[3,5,1,""],MAX_ITERATION_PER_PHASE:[3,5,1,""],MAX_PHASE_LENGTH:[3,5,1,""],POPULATION_SIZE:[3,5,1,""],__init__:[3,3,1,""],_flush_upcoming_SPaTs:[3,3,1,""],_schedule_unserved_vehicles:[3,3,1,""],_set_lane_lane_incidence:[3,3,1,""],_set_non_base_scheduled_arrival:[3,3,1,""],_set_phase_lane_incidence:[3,3,1,""],append_extend_phase:[3,3,1,""],base_badness:[3,3,1,""],cross_over:[3,3,1,""],evaluate_badness:[3,3,1,""],get_optimal_cycle_length:[3,3,1,""],mutate_seq:[3,3,1,""],mutate_timing:[3,3,1,""],set_critical_phase_volumes:[3,3,1,""],solve:[3,3,1,""],update_SPaT:[3,3,1,""]},"src.signal.MinCostFlow_SPaT":{CMIN:[3,5,1,""],LAG:[3,5,1,""],LARGE_NUM:[3,5,1,""],M:[3,5,1,""],__init__:[3,3,1,""],_flush_upcoming_SPaTs:[3,3,1,""],_schedule_unserved_vehicles:[3,3,1,""],_set_lane_lane_incidence:[3,3,1,""],_set_non_base_scheduled_arrival:[3,3,1,""],_set_phase_lane_incidence:[3,3,1,""],append_extend_phase:[3,3,1,""],base_badness:[3,3,1,""],set_critical_phase_volumes:[3,3,1,""],set_dem:[3,3,1,""],solve:[3,3,1,""],update_SPaT:[3,3,1,""]},"src.signal.Pretimed":{LAG:[3,5,1,""],LARGE_NUM:[3,5,1,""],NUM_CYCLES:[3,5,1,""],__init__:[3,3,1,""],_flush_upcoming_SPaTs:[3,3,1,""],_schedule_unserved_vehicles:[3,3,1,""],_set_lane_lane_incidence:[3,3,1,""],_set_non_base_scheduled_arrival:[3,3,1,""],_set_phase_lane_incidence:[3,3,1,""],append_extend_phase:[3,3,1,""],base_badness:[3,3,1,""],set_critical_phase_volumes:[3,3,1,""],solve:[3,3,1,""],update_SPaT:[3,3,1,""]},"src.signal.Signal":{LAG:[3,5,1,""],LARGE_NUM:[3,5,1,""],__init__:[3,3,1,""],_flush_upcoming_SPaTs:[3,3,1,""],_schedule_unserved_vehicles:[3,3,1,""],_set_lane_lane_incidence:[3,3,1,""],_set_non_base_scheduled_arrival:[3,3,1,""],_set_phase_lane_incidence:[3,3,1,""],append_extend_phase:[3,3,1,""],base_badness:[3,3,1,""],set_critical_phase_volumes:[3,3,1,""],update_SPaT:[3,3,1,""]},"src.time_keeper":{TimeKeeper:[1,2,1,""]},"src.time_keeper.TimeKeeper":{__init__:[1,3,1,""],next_sim_step:[1,3,1,""]},"src.trajectory":{FollowerConnected:[4,2,1,""],FollowerConventional:[4,2,1,""],LeadConnected:[4,2,1,""],LeadConventional:[4,2,1,""],Trajectory:[4,2,1,""],earliest_arrival_connected:[4,1,1,""],earliest_arrival_conventional:[4,1,1,""]},"src.trajectory.FollowerConnected":{set_model:[4,3,1,""]},"src.trajectory.FollowerConventional":{solve:[4,3,1,""]},"src.trajectory.LeadConnected":{set_model:[4,3,1,""],solve:[4,3,1,""]},"src.trajectory.LeadConventional":{solve:[4,3,1,""]},"src.trajectory.Trajectory":{discretize_time_interval:[4,3,1,""],set_trajectory:[4,4,1,""]},data:{data:[1,0,0,"-"]},main:{check_py_ver:[0,1,1,""],run_avian:[0,1,1,""]},src:{intersection:[2,0,0,"-"],signal:[3,0,0,"-"],time_keeper:[1,0,0,"-"],trajectory:[4,0,0,"-"]}},objnames:{"0":["py","module","Python module"],"1":["py","function","Python function"],"2":["py","class","Python class"],"3":["py","method","Python method"],"4":["py","staticmethod","Python static method"],"5":["py","attribute","Python attribute"]},objtypes:{"0":"py:module","1":"py:function","2":"py:class","3":"py:method","4":"py:staticmethod","5":"py:attribute"},terms:{"13th":0,"13th16th":0,"16th":0,"42nd40th":0,"abstract":4,"boolean":0,"case":[2,3],"class":[1,2,3,4],"default":3,"final":2,"function":[3,4],"import":2,"int":[0,2],"new":[1,2,3],"return":[1,2,3,4],"short":4,"static":[2,3,4],"true":[2,3],"while":3,EPS:[2,4],For:[0,2,3],Has:3,IDs:0,NOT:[2,3],Not:3,One:3,RES:4,RTS:0,The:[0,1,2,3,4],Use:1,Uses:[2,3,4],__init__:[1,2,3],_do_trj:2,_flush_upcoming_spat:3,_schedule_unserved_vehicl:3,_set_lane_lane_incid:3,_set_non_base_scheduled_arriv:3,_set_phase_lane_incid:3,_trj_point_level:0,_vehicle_level:0,absolut:3,acc:3,acceler:[0,2,4],access:3,accordingli:2,acquir:3,activ:3,actual_green:3,actuat:0,actuatedcontrol:3,add:[0,1,2,3],added:[2,3],addit:0,address:3,adjust:3,advanc:0,after:3,again:1,aim:[0,3],algorithm:[0,3],all:[0,1,2,3],all_serv:2,alloc:3,allow:[2,3,4],allowable_phas:3,alreadi:3,also:[0,2,3],altern:3,amax:[2,4],amin:[2,4],amount:3,ani:[0,2,3,4],anther:3,any_unserved_vehicl:3,append:[2,3],append_extend_phas:3,appendix:0,appli:2,applic:2,approach:0,approxim:4,april:[0,1,2,3,4],arc:3,argument:[0,2],arrai:[2,3,4],arriv:[0,2,4],arrival_dist:4,arrival_tim:4,arxiv:0,assign:[0,2,3,4],assum:[3,4],assumpt:3,attent:0,attribut:4,author:[0,1,2,3,4],autom:[0,2,3],autonom:0,auxiliari:2,avail:[0,3],averag:3,avian:0,avoid:[2,3],award:0,back:3,bad:[0,2,3],badness_accuraci:3,bar:[0,1,2,3,4],base:[0,1,2,3],base_bad:3,beat:3,beaten:3,becaus:3,been:2,befor:[2,3],begin:[0,3],behaviour:[2,4],being:[2,3],bellow:0,best:[3,4],best_spat:3,beta:2,between:[3,4],both:4,bound:3,boundari:3,budget:3,build:4,call:[2,3,4],can:[0,4],cannot:3,cap:3,car:[0,2,4],care:4,carefulli:3,cav:[0,2,4],certain:2,chanc:3,chang:[2,3,4],check:[0,2,3],check_py_v:0,check_trj_redo_need:2,child:3,choos:[0,3],clock:[0,1,2,3],close:[2,4],close_trj_csv:2,closer:2,cmin:3,cnv:[0,2],code:[0,1,2,3],coeffici:[2,4],column:[0,2],com:[0,1,2,3,4],come:2,command:[0,2],complet:3,complete_unserved_vehicl:3,comput:[1,2,3,4],concept:3,condit:3,configur:[2,3],conflict:[1,3],connect:[0,2,4],consecut:4,consid:2,consist:[2,3],constant:3,constraint:4,construct:4,contain:0,context:0,continu:3,control:[0,1,2,3,4],convent:[0,2,4],convert:3,correspond:[2,3],cost:[0,3],could:3,count:3,cover:3,creat:3,criteria:3,critical_volume_ratio:3,cross_ov:3,crossov:3,crossover_s:3,csv:[0,1,2],current:[2,3],curspd:0,cycl:3,cycle_length:3,d_schedul:2,data:[0,2,3,5],date:[0,1,2,3,4],dec:3,deceler:[0,2,4],decid:[2,3],decis:[0,3],decrement_first_unsrvd_indx:2,decrement_last_veh_indx:2,defin:[0,2,3],definit:3,degre:[1,2],delet:2,dep_spe:4,depart:[3,4],departur:[0,2,3,4],departure_tim:2,depend:[2,3],deriv:3,des_spe:2,describ:0,desir:[0,2],desspd:0,dest:[0,2],destin:[0,2],det_id:2,det_rang:2,det_tim:[2,4],det_typ:2,detail:[0,3],detect:[0,1,2,4],determin:3,develop:0,diagram:0,dictionari:[1,2,3],did:3,differ:[3,4],directori:[0,2],discret:[1,4],discretize_time_interv:4,dist:[0,2,4],distanc:[0,1,2,4],do_traj_allow:4,do_traj_comput:0,doe:[2,3,4],done:0,doubl:3,drawn:3,driver:4,due:3,durat:3,each:[0,1,2,3,4],earliest:[2,3,4],earliest_arriv:2,earliest_arrival_connect:4,earliest_arrival_convent:4,eastbound:0,edu:0,either:[0,1,2,3],elaps:[0,2],elapsed_t:2,elefteriad:0,element:3,elit:3,empti:[2,3],end:[0,3,4],end_tim:4,enforc:4,enough:[2,4],enumar:1,enumer:[1,3],enumerate_spat:3,equal:[2,3,4],equat:[2,3],essi:0,estim:4,evaluate_bad:3,even:[1,3],eventu:3,exactli:3,exampl:[0,3],exce:4,except:3,exclus:0,execut:[0,3],exist:2,expir:2,extens:0,extra:4,fact:2,factor:3,fall:3,fals:[2,3,4],far:3,feasibl:3,field:[1,2],file:[0,1,2,3],filenam:0,find:2,first:[0,2,3,4],first_unsrvd_indx:3,fist:2,fit:[0,3],fix:[0,3],florida:0,flow:[0,1,2,3],flush:3,follow:[0,2,3,4],followerconnect:[0,4],followerconvent:[0,4],form:[2,3],format:0,formula:3,forward:1,foundat:0,four:4,from:[0,1,2,3,4],front:4,full:3,fundament:2,fusion:[1,2,3],ga_spat:[0,3],gainesvil:0,gasulla:0,gener:[0,1,2,3,4],genet:[0,3],get:[2,3],get_conflict_dict:1,get_det_rang:2,get_first_detection_tim:2,get_general_param:1,get_max_spe:2,get_min_headwai:2,get_num_lan:2,get_optimal_cycle_length:3,get_phas:1,get_poly_param:2,get_pretimed_paramet:1,get_signal_param:1,get_volum:2,gipp:[0,4],give:3,given:[2,3,4],gmail:[0,1,2,3,4],goal:3,goe:3,goo:3,googl:[0,3],grant:0,green:[1,3,4],green_dur:3,half_max_indx:3,halt:2,happen:[0,3],has:[0,2,3],hash:3,have:[0,2,3],hcm:3,head:[0,3],headwai:[2,3,4],held:2,help:3,henc:3,here:[1,2,3,4],high:3,higher:3,highest:3,hold:3,how:3,howev:[1,3],http:3,imag:0,implement:[0,3],incid:3,includ:[0,1,2,3],incom:[0,2],incomplet:3,increment_first_unsrvd_indx:2,increment_last_veh_indx:2,index:[0,2,3,4,5],indic:2,individu:[0,2,3],indx:2,info:[2,3,4],inform:[0,2],inherit:[3,4],init_tim:2,initi:[1,2,3],input:[0,1,2,3],insid:3,instal:[0,3],instanti:0,instead:3,int_nam:2,intellig:0,intend:[2,3,4],inter_nam:[0,1,2,3],intersect:[0,1,3,5],intersection_nam:0,interv:[1,4],introduct:5,invok:[0,4],is_lead:4,isol:0,its:[2,3,4],jfncvj:3,just:[2,3],keep:[1,2,3],kei:[1,2,3],known:3,lag:[3,4],lambda:3,lane:[0,1,2,3],lane_lane_incid:3,lanes_demand:3,languag:0,larg:3,large_num:3,last:[0,2,3,4],last_veh_arriv:2,late:3,lead:[3,4],lead_arrival_tim:4,lead_det_tim:4,lead_poli:4,lead_veh:4,leadconnect:[0,4],leadconvent:[0,4],least:[2,3],leav:3,left:[0,2],left_par:3,len:3,length:[0,2,3],less:[0,2,3],let:3,level:[2,3],limit:3,line:[0,2],list:[0,2,3],locat:[0,2],log:[0,2],log_at_trj_point_level:[0,2],log_at_vehicle_level:[0,2],longer:3,look:[2,3],loop:0,lower:[2,4],made:3,mahmoud:[0,1,2,3,4],mai:[0,2,3],main:[3,4,5],maintain:4,make:[2,3],manag:2,mani:3,manual:1,map:0,map_veh_type2str:2,martin:0,match:0,matlab:0,matrix:[2,3],max:[1,3,4],max_iteration_per_phas:3,max_num_trajectory_point:2,max_phase_length:3,max_spe:[2,3,4],maxacc:0,maxdec:0,maximum:[0,2,3,4],mcf:0,mean:[2,3],meant:[3,4],measur:[0,2,3],meet:[0,3],merg:3,meter:[2,4],method:[0,2,3,4],methodolog:4,metric:3,min:[1,2,3],min_dist:2,min_dist_to_stop_bar:2,min_headwai:[1,2,3,4],mincostflow_spat:3,minimum:[0,3,4],mode:0,model:[0,2,3,4],modul:[0,5],more:[0,2,3],move:1,movement:[0,3],mpourmehrab:0,multipl:3,must:[0,1,3],mutate_seq:3,mutate_tim:3,name:[0,2],nation:0,necessari:[2,3,4],need:[1,2,4],neg:[0,4],network:0,never:3,newli:3,next:3,next_sim_step:1,nit:3,node:3,non:1,none:2,normal:[2,3],northbound:0,note:[0,3,4],now:[0,3],num_cycl:3,num_lan:[2,3],num_serv:2,number:[0,1,2,3,4],numer:2,numpi:4,object:[1,2,3,4],odd:1,off:3,omit:4,onc:[2,3],one:[0,1,2,3,4],ones:3,ongo:3,onli:[0,2,3,4],oper:3,opposit:3,optim:[0,1,2,3,4],optimization_algo:0,option:4,order:2,organ:0,origin:2,other:[2,3],otherwis:[2,3],ots:2,output:[0,2],outsid:3,overestim:3,overrid:4,overview:5,overwrit:3,own:4,packag:[0,3],page:5,pai:0,param:[3,4],paramet:[0,1,2,3,4],parent:3,part:[0,4],particular:4,pass:3,per:[0,2,3],perform:[0,3],perman:3,peter:4,phase:[0,1,5],phase_length:3,phase_seq:3,physic:0,pick:3,pip3:0,pipfil:3,place:[3,4],plan:[0,2,4],planer:4,planner:0,pli:3,plu:[0,3],point:[0,2,3,4],pointer:2,pointless:2,poly_coeff:2,polynomi:[1,2,4],popul:3,population_s:3,possibl:[0,3],pourmehrab:[0,1,2,3,4],practic:3,pre:[1,3],prealloc:2,prefer:[0,1,3],prefix:0,preprint:0,present:3,pretim:[0,3],print:[0,2],print_clock:0,print_departur:[0,2],print_detect:[0,2],print_signal_detail:[0,3],print_trj_info:0,print_trj_point:2,prior:3,problem:[2,3],profil:[2,4],program:[0,2,3],project:[0,1],provid:[0,1,2,3],purg:3,purge_served_veh:2,purpos:2,pyenv:3,python:[0,3,5],qualifi:3,qualiti:[2,3],queue:2,radio:2,random:3,randomli:3,rang:[1,2],ranka:0,rate:[0,2,3],reach:4,real:3,realtim:0,reason:3,record:2,red:[1,3],redo_trj_allow:3,reduc:3,refer:[0,2,3,4],refin:1,regener:4,remain:3,remov:[0,2,3],replac:3,repres:2,request:2,requir:[0,1,3],research:4,reserv:[0,3],reset:[2,3],reset_first_unsrvd_indx:2,reset_trj_point:2,resolut:[1,4],respect:[2,3],rest:3,restrict:3,result:3,right:[0,2],right_par:3,road:0,roll:3,row:2,rule:1,run:[0,2,3,4],run_avian:0,s_schedul:2,safe:4,safeti:3,same:3,sampl:3,satur:3,save:[0,2],save_csv:2,scenario:[0,2],schedul:[0,2,3,4],scheduled_arriv:[2,3],scheduled_departur:3,scienc:0,script:5,search:5,sec:[2,4],second:[0,1,2,3,4],see:0,select:3,self:3,seq:3,sequenc:[1,3],serv:[0,2,3],serve_update_at_stop_bar:2,served_vehicle_tim:3,set:[0,1,2,3,4],set_critical_phase_volum:3,set_dem:3,set_departure_time_for_csv:2,set_earliest_arriv:2,set_elapsed_sim_tim:2,set_first_trj_point_indx:2,set_last_trj_point_indx:2,set_model:4,set_poly_coeff:2,set_scheduled_arriv:2,set_trajectori:4,sever:3,shall:[0,2,4],shape:2,should:[0,2,3],show:[0,2,3],signal:[0,1,2,5],sim_start:1,similar:3,simpli:[3,4],simplif:3,simul:[0,2,3,4,5],simulation_tim:2,simultan:3,sinc:3,singl:3,sink:3,size:4,skip:[3,4],slack:3,slove:3,small:4,solarpark:0,solv:[3,4],solver:3,some:[0,2,3],sometim:3,sort:[2,3],sourc:2,southbound:0,space:2,spat:[0,5],specif:4,specifi:[2,3],speed:[0,1,2,3,4],split:[1,3],spot:3,springhil:0,src:[0,1,2,3,4],stamp:[0,2,4],start:[0,1,2,3,4],start_tim:4,state:2,statu:3,step:[1,3],stop:[0,1,2,3,4],store:[0,3],str:0,structur:2,subclass:4,subset:3,sum:3,support:[0,2,3],sure:[2,3],t_earliest:[2,4],t_schedul:2,take:3,tallahasse:0,task:3,teh:3,tempor:3,temporari:3,tend:4,terl:0,term:5,termin:[0,3],test:0,test_tim:0,texa:0,than:[2,3,4],thei:[2,3],them:[2,3],theori:[1,3],thi:[1,2,3,4],third:2,those:[2,3],threshold:2,through:[0,2,3],throughput:3,thu:2,till:4,time:[0,1,2,4,5],time_keep:1,time_split:3,time_threshold:[2,3],timekeep:1,todo:1,too:[0,1],track:2,traffic:[0,1,2,3],trajectori:[0,1,2,3,5],translat:2,transport:4,travel:[2,3],tri:3,tripl:0,tupl:3,turn:[0,2],two:[2,3,4],txt:0,type:[0,2],ufl:0,uml:0,under:[0,2,3,4],underdevelop:3,underscor:2,unit:[0,2,3],univers:0,unknown:3,unless:3,unlik:3,unserv:[2,3],unservd:2,updat:[0,2,3,4],update_spat:3,update_vehicles_info:2,use:3,used:[0,1,2,3],useful:2,user:2,using:[0,3,4],util:3,valid:[0,2,3],valu:[1,2,3],variabl:[0,3],variant:3,varieti:[0,3],vector:2,veh:[2,3,4],veh_indx:2,vehicl:[0,2,3,4],vehlist:2,ver:0,version:0,visit:0,volum:[2,3],wai:3,want:[0,2,3,4],websit:0,weight:3,were:3,westbound:0,what:[2,3],when:[0,2,3,4],where:[0,1,2,3],which:[0,2,3],whole:4,within:3,without:[3,4],work:[0,3],workflow:0,wors:3,worst:3,would:[2,3],write:2,written:[3,4],yellow:[1,3],yet:2,you:[0,1],zero:[1,2,3,4],zoom:0},titles:["1. Overview","2. Simulator/Data","3. Intersection","4. Signal Phase and Timing (SPaT)","5. Trajectory","Welcome to AVIAN\u2019s documentation!"],titleterms:{avian:5,data:1,document:5,indic:5,intersect:2,introduct:0,main:0,overview:0,phase:3,python:1,script:0,signal:3,simul:1,spat:3,tabl:5,term:0,time:3,todo:3,trajectori:4,welcom:5}})