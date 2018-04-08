const EventEmitter=require('events').EventEmitter;
let ev=new EventEmitter();
ev.running=false;
let popen=require('child_process');

popen.run=function(cmd){
	let args=arguments;
	let proc=popen.exec(cmd,{env:process.env});
	ev.running=false;
	ev.cin=function(data){ proc.stdin.write(data+'\n');}
	let stm=setTimeout(function(){
		ev.running=true;
		ev.emit('start');
	},3000);
	proc.stdout.on('data',function(data){
		ev.emit('cout',data);
	});
	proc.stderr.on('data',function(data){
		ev.emit('cerr',data);
	});
	proc.on('close',function(code){
		console.log('rosrun closed:'+code);
		if(ev.running){
			ev.running=false;
			ev.emit('stop');
		}
		else clearTimeout(stm);
		setTimeout(function(){
			popen.run.apply(popen,args);
		},3000);
	});
	return ev;
}

module.exports=popen;