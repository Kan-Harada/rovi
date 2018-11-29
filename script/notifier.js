const ros = require('rosnodejs');
const EventEmitter = require('events').EventEmitter;

function calcHash(text) {
    'use strict';

    var hash = 5381,
        index = text.length;

    while (index) {
        hash = (hash * 33) ^ text.charCodeAt(--index);
    }

    return hash >>> 0;
}

class Notifier extends EventEmitter{
  constructor(ros,ns){
    super();
    this.ros=ros;
    this.ns=ns;
    this.hashes={};
    this.objs={};
    this.tid=null;
    this.enable=false;
    this.init();
  }
  async init(){
    try{
      this.objs=await this.ros.getParam(this.ns);
      return true;
    }
    catch(err){
      console.log('Init Param err:'+this.ns);
      return false;
    }
  }
  check(param){
    let keys=Object.keys(param);
    if(this.enable){
      if(keys.length>0){
        let key=keys[0];
        let val=param[key];
        let hash=calcHash(JSON.stringify(val));
        if(! this.hashes.hasOwnProperty(key) || this.hashes[key]!=hash){
          this.hashes[key]=hash;
          this.objs[key]=val;
          this.emit('change',key,val);
        }
        delete param[key];
        const who=this;
        setImmediate(function(){ who.check(param);});
      }
      else{
        const who=this;
        this.tid=setTimeout(async function(){
          who.tid=null;
          let param;
          try{
            param=await who.ros.getParam(who.ns);
          }
          catch(err){
            console.log('getParam err:'+who.ns);
            who.stop();
          }
          who.check(param);
        },1000);
      }
    }
  }
  push(param){
    for(let key in param){
      let val=param[key];
      this.emit('change',key,val);
    }
  }
  async start(){
    if(this.enable) return;
    if(! await this.init()) return;
    this.enable=true;
    for(let key in this.objs){
      let val=this.objs[key];
      let hash=calcHash(JSON.stringify(val));
      if(! this.hashes.hasOwnProperty(key) || this.hashes[key]!=hash){
        this.hashes[key]=hash;
        this.emit('change',key,val);
      }
    }
    this.emit('start');
    this.check({});
  }
  stop(){
    if(this.enable){
      this.enable=false;
      if(this.tid!=null) clearTimeout(this.tid);
      this.emit('stop');
    }
  }
  reset(){
    this.stop();
    this.hashes={};
    this.objs={};
  }
  diff(obj){
    let c={};
    for(let k in obj){
      if (this.objs.hasOwnProperty(k)){
        if(this.objs[k]!=c[k]) c[k]=this.objs[k];
      }
    }
    return c;
  }
}

module.exports=Notifier
