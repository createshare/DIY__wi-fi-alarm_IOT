<?php

	//Memcache��SAEΪ�������ṩ�ķֲ�ʽ�ڴ滺����������Թ���ķ�ʽ�����û���С���ݡ�
	//Memcache��Ҫ��ʹ�ó���������������
	//		��Ҫ����ĳЩ key-value ��ʽ��С����ʱ������ΪSAE��Web�����Ƿֲ�ʽ����������ʹ��ȫ�ֱ�����ʽ�ȷ�ʽ�ǲ��еģ���
	//    ����MySQL�Ⱥ�˴洢�����ݡ����ٽ���������Ӧ�������˴洢��ѹ����
	//�û���Ҫ�������߹���ƽ̨����Memcache��Ȼ��ſ���ͨ��API��дMemcache��
	//memcache_init - ��ʼ��MC����
$mmc = new Memcache;  //����}
$mmc->connect('localhost', 11211) or die ("Could not connect");


//�ж��ǲ��ǡ�01�š���������POST���͹����ı�����Ϣ
if (isset($_POST['alarmState01'])) 	//isset �� �������Ƿ����� //�� PHP �У�Ԥ����� $_POST ���������ռ����� method="post" �ı��е�ֵ��
{
	$alarmState01 = $_POST['alarmState01'];  

		//memcache_set - ����MC����
	$mmc->set("alarmState01", $alarmState01);
	echo 'ok';
}

//�ж��ǲ��ǡ�02�š���������POST���͹����ı�����Ϣ
if (isset($_POST['alarmState02'])) 	
{
	$alarmState02 = $_POST['alarmState02'];  

		//memcache_set - ����MC����
	$mmc->set("alarmState02", $alarmState02);
	echo 'ok';
}

//�ж��ǲ��ǡ�03�š���������POST���͹����ı�����Ϣ
if (isset($_POST['alarmState03'])) 	
{
	$alarmState03 = $_POST['alarmState03'];  

		//memcache_set - ����MC����
	$mmc->set("alarmState03", $alarmState03);
	echo 'ok';
}

//����GET����������Http�ͻ��˷�������
//else
//{
		//memcache_get - ��ȡMC����
	echo $mmc->get("alarmState01") . "\r\n" . $mmc->get("alarmState02") . "\r\n" . $mmc->get("alarmState03");
//}

?>