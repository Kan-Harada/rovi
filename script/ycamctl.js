#!/usr/bin/env node

const NSrovi = process.env.ROS_NAMESPACE;
const NSycamctrl = NSrovi+'/ycam_ctrl';
const NSps = NSrovi+'/pshift_genpc';
const NSgenpc = NSrovi+'/genpc';
const NScamL = NSrovi+'/left';
const NScamR = NSrovi+'/right';
const NSlive = NSrovi+'/live';
const ros = require('rosnodejs');
const std_msgs = ros.require('std_msgs').msg;
const std_srvs = ros.require('std_srvs').srv;
const rovi_srvs = ros.require('rovi').srv;
const rovi_msgs = ros.require('rovi').msg;
const EventEmitter = require('events').EventEmitter;
const jsyaml = require('js-yaml');
const Notifier = require('./notifier.js');
const sensor_msgs = ros.require('sensor_msgs').msg;

// ycam3s.jsでYCAM3Dを選択
const sens = require('./'+process.argv[2]+'.js')
//画像処理機能
const ImageSwitcher = require('./image_switcher.js');
//カメラ制御・イベント受信機能
const SensControl = require('./sens_ctrl.js');
//ライブ時のfpsを設定
const liveFps=10;
//位相シフト撮影枚数
const psCnt=13;
//ライブON/OFF
const Livef=1;

function sleep(msec){return new Promise(function(resolve){setTimeout(function(){resolve(true);},msec);});}
function add_sendmsg(pub){
  pub.sendmsg=function(s){
    let m=new std_msgs.String();
    m.data=s;
    pub.publish(m);
  }
}
process.on('exit',async function() {
  ros.log.info('ycamctl exiting...');
  await sleep(1000);
});

setImmediate(async function() {
//=============
// 初期化部
//=============
  const rosNode = await ros.initNode(NSycamctrl);
  const image_L = new ImageSwitcher(rosNode, NScamL);
  const image_R = new ImageSwitcher(rosNode, NScamR);
//---------publisher and subscriber 1/2
  const pub_stat = rosNode.advertise(NSrovi + '/stat', std_msgs.Bool);
  const pub_error = rosNode.advertise('/error', std_msgs.String);
  add_sendmsg(pub_error);
  const pub_info = rosNode.advertise('/message', std_msgs.String);
  add_sendmsg(pub_info);
  const pub_pcount=rosNode.advertise(NSrovi+'/pcount',std_msgs.Int32);
  const pub_Y1=rosNode.advertise(NSrovi+'/Y1',std_msgs.Bool);
  const genpc=rosNode.serviceClient(NSgenpc, rovi_srvs.GenPC, { persist: true });
  if (!await rosNode.waitForService(genpc.getService(), 2000)) {
    ros.log.error('genpc service not available');
    return;
  }
  const param_camnode=await rosNode.getParam(NSrovi + '/camera');
  let param={
    camps: new Notifier(rosNode,NSps + '/camera'),//Genpc mode camera params
    camlv: new Notifier(rosNode,NSlive + '/camera'),  //Live mode camera params
    proj: new Notifier(rosNode,NSps + '/projector') //Genpc projector params
  };

//=============
// イベント処理部
//=============
  param.camlv.on('change',async function(key,val){
    let obj={};
    obj[key]=val;
    await sens.cset(obj);
  });
  param.proj.on('change',async function(key,val){
    let obj={};
    obj[key]=val;
    await sens.pset(obj);
    if(key!='Go' && key!='Mode2'){
      if(sensEv.streaming){
        await sensEv.scanStop(1000);
        sensEv.lit=false;
        sensEv.scanStart(1000);
     }
    }
  });
  let sensEv=await sens.open(rosNode, NSrovi, param_camnode);
  sensEv=SensControl.assign(sensEv);
// 起動時処理  
  sensEv.on('wake', async function() {
    for(let n in param) await param[n].start();
    sensEv.fps=liveFps;
// プロジェクター外部トリガ無効
    let mode2={}; mode2['Mode2']=0;
    await sens.pset(mode2);
    await sleep(100);
// プロジェクターに位相シフト撮影パターンを設定
    let pat1={}; pat1['Mode']=1;
    await sens.pset(pat1);
    ros.log.warn('NOW ALL READY ');
    pub_info.sendmsg('YCAM ready');
    if(Livef) {
      sensEv.lit=false;
      sensEv.scanStart(3000);
    }
    else {
      sens.normal=true;
      pslock=false;
    }

  });
  sensEv.on('stat', function(f){
    let m=new std_msgs.Bool();
    m.data=f;
    pub_stat.publish(m);
  });
  sensEv.on('shutdown', async function(){
    ros.log.info('ycam down '+sens.cstat+' '+sens.pstat);
    for(let n in param) param[n].reset();
    rosNode.setParam(NSrovi+'/camera',param_camnode);
    pub_error.sendmsg('YCAM disconected');
  });
  sensEv.on('left', async function(img,ts){
    image_L.emit(img,ts,sensEv.lit);
  });
  sensEv.on('right', async function(img,ts){
    image_R.emit(img,ts,sensEv.lit);
  });
  sensEv.on('trigger', async function() {
    sens.cset({'CaptureCnt': 0x01});
  });
  sensEv.on('timeout', async function() {
    ros.log.error('Image streaming timeout');
    pub_error.sendmsg('Image streaming timeout');
    sens.kill();
  });

//=============
// サービスの定義
//=============
  let pslock=true;
  sensEv.on('trigger',function(){ pslock=false; });
  sensEv.on('shutdown',function(){ pslock=true; });
//ライブ
  let ps2live = function(tp){ //---after "tp" msec, back to live mode
    image_L.thru();
    image_R.thru();
    pslock=false;
    if(!Livef) return false;
    setTimeout(function(){
      sensEv.scanStart();
    },tp);
    param.camlv.raise(param.camlv.diff(param.camps.objs));//---restore overwritten camera params
  }

//位相シフト撮影
  let psgenpc = function(req,res){
    if(!sens.normal){
      ros.log.warn(res.message='YCAM not ready');
      res.success = false;
      return true;
    }
    if(pslock){
      ros.log.warn(res.message='genpc busy');
      res.success = false;
      return true;
    }
    return new Promise(async (resolve) => {
      pslock=true;
      await sensEv.scanStop(1000); //---wait stopping stream with 1000ms timeout
      ros.log.info('Streaming stopped');

      let obj={};
      obj['Mode2']=1;
      await sens.pset(obj);
      await sens.cset(param.camps.objs); //---overwrites genpc camera params
      let wdt=setTimeout(async function() { //---start watch dog
        ps2live(1000);
        const errmsg = 'pshift_genpc timed out';
        pub_error.sendmsg(errmsg);
        res.success = false;
        res.message = errmsg;
        pub_Y1.publish(new std_msgs.Bool());
        resolve(true);
      }, param.proj.objs.Interval*psCnt + 1000);
//for monitoring
     let icnt=0;
     image_L.hook.on('store',function(img,t2){
       let t1=img.header.stamp;
       ros.log.info(('00'+icnt.toString(10)).substr(-2)+' '+(t1.nsecs*1e-9+t1.secs)+' '+(t2.nsecs*1e-9+t2.secs));
       icnt++;
     });

      ros.log.info('Ready to store');
      setTimeout(function(){
          if(psCnt==13) sens.cset({'CaptureCnt': 0x0d});
          else if(psCnt==4) sens.cset({'CaptureCnt': 0xfff2000d});
//           sens.cset({'CaptureCnt': 0xfdf2020d});
        },
        95
      );
      await sleep(100);
      let imgs;
      try{
        imgs=await Promise.all([image_L.store(psCnt),image_R.store(psCnt)]); //---switch to "store" mode
        obj['Mode2']=0;
        await sens.pset(obj);
      }
      catch(err){
        const msg="image_switcher::exception";
        ps2live(1000);
        ros.log.error(msg);
        pub_error.sendmsg(msg);
        res.success = false;
        res.message = msg;
        resolve(true);
        pub_Y1.publish(new std_msgs.Bool());
        return;
      }
      clearTimeout(wdt);
      if(psCnt==13) {
        let gpreq = new rovi_srvs.GenPC.Request();
        gpreq.imgL = imgs[0];
        gpreq.imgR = imgs[1];
        let gpres;
        try {
          ros.log.info("call genpc");
          gpres = await genpc.call(gpreq);
          ros.log.info("ret genpc");
          res.message = imgs[0].length + ' images scan complete. Generated PointCloud Count=' + gpres.pc_cnt;
          res.success = true;
        }
        catch(err) {
          res.message = 'genpc failed';
          res.success = false;
        }
        let pcount=new std_msgs.Int32();
        pcount.data=gpres.pc_cnt;
        pub_pcount.publish(pcount);
        let tp=Math.floor(gpres.pc_cnt*0.0001)+10;
        ros.log.info('Time to publish pc '+tp+' ms');
      }
      ps2live(1000);
      let finish=new std_msgs.Bool();
      finish.data=res.success;
      pub_Y1.publish(finish);
      resolve(true);
    });
  }

//---------publisher and subscriber 2/2
  const svc_X1=rosNode.advertiseService(NSps, std_srvs.Trigger, psgenpc);
  const sub_X1=rosNode.subscribe(NSrovi+'/X1',std_msgs.Bool,async function(){
    if (!sens.normal){
      pub_error.sendmsg('request cancelled due to YCAM status');
      pub_Y1.publish(new std_msgs.Bool());
      return;
    }
    let req=new std_srvs.Trigger.Request();
    let res=new std_srvs.Trigger.Response();
    let ret=psgenpc(req,res);
//  if(typeof(ret)=='boolean'){ //request refused
//    pub_Y1.publish(new std_msgs.Bool());
//  }
  });
  const svc_reset = rosNode.subscribe(NSrovi+'/reset',std_msgs.Bool,async function(){
    await sens.reset();
  });
});
