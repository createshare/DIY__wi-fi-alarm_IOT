<?php

	//Memcache是SAE为开发者提供的分布式内存缓存服务，用来以共享的方式缓存用户的小数据。
	//Memcache主要的使用场景有以下两个：
	//		需要共享某些 key-value 形式的小数据时。（因为SAE的Web服务是分布式环境，所以使用全局变量方式等方式是不行的）。
	//    缓存MySQL等后端存储的数据。快速进行数据响应，减轻后端存储的压力。
	//用户需要先在在线管理平台创建Memcache，然后才可以通过API读写Memcache。
	//memcache_init - 初始化MC链接
$mmc = new Memcache;  //连接}
$mmc->connect('localhost', 11211) or die ("Could not connect");


//判断是不是“01号”报警器，POST发送过来的报警信息
if (isset($_POST['alarmState01'])) 	//isset — 检测变量是否设置 //在 PHP 中，预定义的 $_POST 变量用于收集来自 method="post" 的表单中的值。
{
	$alarmState01 = $_POST['alarmState01'];  

		//memcache_set - 存入MC数据
	$mmc->set("alarmState01", $alarmState01);
	echo 'ok';
}

//判断是不是“02号”报警器，POST发送过来的报警信息
if (isset($_POST['alarmState02'])) 	
{
	$alarmState02 = $_POST['alarmState02'];  

		//memcache_set - 存入MC数据
	$mmc->set("alarmState02", $alarmState02);
	echo 'ok';
}

//判断是不是“03号”报警器，POST发送过来的报警信息
if (isset($_POST['alarmState03'])) 	
{
	$alarmState03 = $_POST['alarmState03'];  

		//memcache_set - 存入MC数据
	$mmc->set("alarmState03", $alarmState03);
	echo 'ok';
}

//用于GET请求，向其他Http客户端发送数据
//else
//{
		//memcache_get - 获取MC数据
	echo $mmc->get("alarmState01") . "\r\n" . $mmc->get("alarmState02") . "\r\n" . $mmc->get("alarmState03");
//}

?>