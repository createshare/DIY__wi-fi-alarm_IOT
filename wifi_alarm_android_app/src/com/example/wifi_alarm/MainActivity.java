package com.example.wifi_alarm;

import android.app.Activity;
import android.app.Service;


//ע������һ�㲻�Լ�����menu����Ϊÿ��ActivityĬ�϶��Դ���һ��������Ҫ������Ϊ���Ӳ˵������Ӧ�˵���ĵ���¼�(�����ص�����)��
//OnCreateOptionsMenu()   ���ڴ����˵���
//OnOptionsMenuSelected() ������Ӧ�˵���ĵ��

import android.util.Log;
//ÿ��activity����һ���˵���һ���˵����ܰ�������˵���Ͷ���Ӳ˵����Ӳ˵���ʵҲ�ǲ˵�����Ϊ��ʵ����Menu�ӿڣ���
//����Ӳ˵�Ҳ���԰�������˵��
import android.view.Menu; //android.view.Menu�ӿڴ���һ���˵���android������������ֲ˵��
import android.view.MenuItem; //android.view.MenuItem����ÿ���˵���  //android.view.SubMenu�����Ӳ˵�
import android.view.View.OnTouchListener;

import android.view.View;  //����  android.view.view(android��һ����װ��)�������Ϳ�����view�еķ�����
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.EditText;

import android.view.MotionEvent;
import android.view.inputmethod.InputMethodManager;
import android.view.ViewGroup;

import java.util.Calendar;
import java.util.Timer;
import java.util.TimerTask;

import android.os.Bundle;
import android.os.Handler;
import android.os.Message;

//import android.text.format.Time; 
import android.graphics.Color;


/////////////////////////////////////////////
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.HttpClient;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicNameValuePair;
import org.apache.http.protocol.HTTP;
import org.apache.http.util.EntityUtils;
import org.json.JSONArray;
import org.json.JSONObject;


import java.util.ArrayList;
import java.util.List;

import android.os.Vibrator;

import android.media.AudioManager;
import android.media.MediaPlayer;


//���ݴ洢
import android.content.Context;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;

//һ��Activity��һ��Ӧ�ó���������ṩһ����Ļ���û���������Ϊ�����ĳ���������粦�š����ա�����email������ͼ��
//ÿһ��activity������һ�����ڣ���������Ի����û��ӿڡ�����ͨ��������Ļ����Ҳ����С����Ļ��������������֮�ϡ�
//һ��Ӧ�ó���ͨ���ɶ��activities��ɣ�����ͨ��������Ϲ�ϵ��
//ͨ����һ��Ӧ�ó����е�activity��ָ��Ϊ"main"activity������һ������Ӧ�ó����ʱ����ָ��û����Ǹ�activity��
//ÿһ��activityȻ�����������һ��activityΪ����ɲ�ͬ�Ķ�����
//ÿһ��һ��activity������ǰһ��activity��ֹͣ�ˣ�����ϵͳ����activity��һ��ջ�ϣ���back stack������
//��һ����activity�������������͵�ջ����ȡ���û����㡣Back Stack���ϼ򵥡�����ȳ���ԭ��
//���ԣ����û���ɵ�ǰactivityȻ����back��ť����������ջ�����ұ��ݻ٣���Ȼ��֮ǰ��activity�ָ���

public class MainActivity extends Activity 
{
	// ����һ������ͼƬ������
	int[] imageIds = new int[]{  
			R.drawable.black,
			R.drawable.white,
			R.drawable.red,
			R.drawable.green,
			R.drawable.blue,
			R.drawable.yellow,
			R.drawable.green_normal,
			R.drawable.red_alarm,
	};	

	ImageView  alarmImage01;   //01�ű���������Ӧ��"״̬ͼƬ"
	ImageView  alarmImage02;  //02�ű���������Ӧ��"״̬ͼƬ"
	ImageView  alarmImage03;  //03�ű���������Ӧ��"״̬ͼƬ"

	int year;      
	int month;      
	int day;    

	int hour; 
	int minute;    
	int sec;


	TextView ID01_timeText01;  //01�ű�����:������
	TextView ID01_timeText02;  //01�ű�����:ʱ����
	
	TextView ID02_timeText01;  //02�ű�����:������
	TextView ID02_timeText02;  //02�ű�����:ʱ����
	
	TextView ID03_timeText01;  //03�ű�����:������
	TextView ID03_timeText02;  //03�ű�����:ʱ����
					
	AudioManager aManager;  //�������
	Vibrator vibrator;   //�����



	
	HttpClient httpClient;	
	String[] alarmState = new String[10];



	

	int workOnlyOnce = 1;   //ֻ�ڴ�APPʱ��ʹ��һ��
	
	int workstate = 0;      //ϵͳ״̬����  // ״̬��
	//ֵΪ1ʱ��ʹ�ܱ�����������ʾ���Դ���һ�α������� �����±���ʱ�䡢����������ʾ��
	//ֵΪ0ʱ����ʾ���Բ����б��������������ظ������������� 
	//int enAlarmTimeChange = 0; 
	
	String account;
	String code;
	
	String name_ID01;	//01�ű��������Ե���Ʒ����
	String name_ID02;	//02�ű��������Ե���Ʒ����
	String name_ID03;	//03�ű��������Ե���Ʒ����
	EditText readNameEdit ;  //ר�Ŷ�ȡ:���ڶ�ȡ"�û��ļ�"������������
	EditText writeNameEdit ;  //ר��д��:���ڶ�ȡ"�û��ļ�"������������




	

		
	
	//Activity���������ڵ�һ����������
  @Override
  protected void onCreate(Bundle savedInstanceState)
  {
  	//super���ǵ��ø�������Ի��߷�������֮this���ǵ��ñ����е����Ժͷ���
  	//super.onCreate(savedInstanceState)�ǵ��ø����onCreate���캯��
  	//savedInstanceState�Ǳ��浱ǰActivity��״̬��Ϣ
  	//onCreate�����Ĳ�����һ��Bundle���͵Ĳ���
  	//Bundle���͵�������Map���͵��������ƣ�������key-value����ʽ�洢���ݵġ�
	  super.onCreate(savedInstanceState);
	  // ʹ��activity_main.xml�ļ�����Ľ��沼��
	  setContentView(R.layout.activity_main);
	  
  
	 // ����һ����ʱ�����øü�ʱ�������Ե�ִ��ָ������
	 new Timer().schedule(new TimerTask()
	 {
		@Override
		public void run()
		{
			//��ʱ����,��ʱ����message  
			Message msgTiming = new Message(); 
			msgTiming.what = 2;   
			myHandler.sendMessage(msgTiming);  //����message 
		}
	 }, 0, 1000); //1000��ʾ��1000ms������ʱÿ 1 ��ִ��һ�� 	
		
		  
	// ��ȡϵͳ����Ƶ����
	aManager = (AudioManager) getSystemService(
		Service.AUDIO_SERVICE);


	// ��ȡϵͳ��Vibrator����
	vibrator = (Vibrator) getSystemService(
		Service.VIBRATOR_SERVICE);

	alarmImage01 = (ImageView) findViewById(R.id.imageView1);   //01�ű���������Ӧ��"״̬ͼƬ"
	alarmImage02 = (ImageView) findViewById(R.id.imageView2);  //02�ű���������Ӧ��"״̬ͼƬ"
	alarmImage03 = (ImageView) findViewById(R.id.imageView3);  //03�ű���������Ӧ��"״̬ͼƬ"


	ID01_timeText01 =  (TextView) findViewById(R.id.ID01_timeText01);  //01�ű�����:������
	ID01_timeText02 =  (TextView) findViewById(R.id.ID01_timeText02);  //01�ű�����:ʱ����
	
	ID02_timeText01 =  (TextView) findViewById(R.id.ID02_timeText01);  //02�ű�����:������
	ID02_timeText02 =  (TextView) findViewById(R.id.ID02_timeText02);  //02�ű�����:ʱ����
	
	ID03_timeText01 =  (TextView) findViewById(R.id.ID03_timeText01);  //03�ű�����:������
	ID03_timeText02 =  (TextView) findViewById(R.id.ID03_timeText02);  //03�ű�����:ʱ����
		
	 
	 // ����DefaultHttpClient����
	 httpClient = new DefaultHttpClient();

	 

//SharedPreferencesҲ��һ�����͵����ݴ洢��ʽ�����ı����ǻ���XML�ļ��洢key-value��ֵ�����ݣ�ͨ�������洢һЩ�򵥵�������Ϣ��
//����Context�����getSharedPreferences()������õ�SharedPreferences������Ա�ͬһӦ�ó����µ������������.
//��洢λ����/data/data/<����>/shared_prefsĿ¼�¡�
//SharedPreferences������ֻ�ܻ�ȡ���ݶ���֧�ִ洢���޸ģ��洢�޸���ͨ��Editor����ʵ�֡�
//ʵ��SharedPreferences�洢�Ĳ������£� һ������Context��ȡSharedPreferences����       ��������edit()������ȡEditor����
//                                  ����ͨ��Editor����洢key-value��ֵ�����ݡ�    �ġ�ͨ��commit()�����ύ���ݡ�
//Context.MODE_PRIVATEΪĬ�ϲ���ģʽ,������ļ���˽������,ֻ�ܱ�Ӧ�ñ������,�ڸ�ģʽ��,д������ݻḲ��ԭ�ļ�������
//��δ���ִ�й��󣬼���/data/data/com.test/shared_prefsĿ¼��������һ��user.xml�ļ���һ��Ӧ�ÿ��Դ������������xml�ļ���
	 
//��SharedPreferences��ȡ����: 
SharedPreferences preferences=getSharedPreferences("user", Context.MODE_PRIVATE);
//���account�����ڣ��򷵻�ֵΪ"0"  //account=preferences.getString("account", "none"); ���account�����ڣ��򷵻�ֵΪ"none" 
account=preferences.getString("account", "0");
code=preferences.getString("code", "0");


name_ID01 = preferences.getString("name_ID01", "0");  //01�ű��������Ե���Ʒ����
name_ID02 = preferences.getString("name_ID02", "0");  //02�ű��������Ե���Ʒ����
name_ID03 = preferences.getString("name_ID03", "0");  //03�ű��������Ե���Ʒ����


readNameEdit = (EditText) findViewById(R.id.nameEdit01); //��ʾ//01�ű��������Ե���Ʒ����
readNameEdit.setText(name_ID01);

readNameEdit= (EditText) findViewById(R.id.nameEdit02);//��ʾ//02�ű��������Ե���Ʒ����
readNameEdit.setText(name_ID02);

readNameEdit= (EditText) findViewById(R.id.nameEdit03);//��ʾ//03�ű��������Ե���Ʒ����
readNameEdit.setText(name_ID03);


	//��android�����ͨ�����edittext֮��Ĳ���,ʹ���������
	setupUI(findViewById(R.id.mWindow));
  }

  
//Android��activity�Ѿ�Ϊ������ǰ��������android.view.Menu����
//��ֻ��Ҫ��д�����onCreateOptionMenu����������onOptionsItemSelected�������ɡ�
//���ṩ�˻ص�����onCreateOptionsMenu(Menu menu)�����ǳ�ʼ���˵������ݡ�
//�÷���ֻ����ѡ��˵���һ����ʾ��ʱ��ִ�У�
//�������Ҫ��̬�ı�ѡ��˵������ݣ���ʹ�� onPrepareOptionsMenu(Menu)��
  
  //ע������һ�㲻�Լ�����menu����Ϊÿ��ActivityĬ�϶��Դ���һ��������Ҫ������Ϊ���Ӳ˵������Ӧ�˵���ĵ���¼�(�����ص�����)��
  //OnCreateOptionsMenu()   ���ڴ����˵���
  // �˷������ڳ�ʼ���˵�������menu�������Ǽ���Ҫ��ʾ��Menuʵ��������true����ʾ��menu,false ����ʾ;
  //(ֻ���ڵ�һ�γ�ʼ���˵�ʱ����)   
  @Override
  public boolean onCreateOptionsMenu(Menu menu) 
  {
      // Inflate the menu; this adds items to the action bar if it is present.
  	//��Activity���еĺ���getMenuInflater()�����������Activity��MenuInflater��
  	//��ͨ��MenuInflater����������menu XML���menu��Ϊ��Activity�Ĳ˵���
      getMenuInflater().inflate(R.menu.main, menu);
      return true;
  }

  //OnOptionsMenuSelected() ������Ӧ�˵���ĵ��
  //�˵�����ʱ���ã�Ҳ���ǲ˵���ļ���������
  @Override
  public boolean onOptionsItemSelected(MenuItem item) 
  {
      // Handle action bar item clicks here. The action bar will
      // automatically handle clicks on the Home/Up button, so long
      // as you specify a parent activity in AndroidManifest.xml.
      int id = item.getItemId();
      
      //��Ӧÿ���˵���(ͨ���˵����ID)
      if (id == R.id.action_settings) {
      	//����true��ʾ������˵�����¼�������Ҫ�����¼�����������ȥ��
          return true;
      }
      
      //��û�д�����¼�����������������
      return super.onOptionsItemSelected(item);
  }
   
  
  
  
  
//�ȶ���һ����������̵Ĺ����෽��
public static void hideSoftKeyboard(Activity activity) 
{
	InputMethodManager inputMethodManager = (InputMethodManager)  activity.getSystemService(Activity.INPUT_METHOD_SERVICE);
	inputMethodManager.hideSoftInputFromWindow(activity.getCurrentFocus().getWindowToken(), 0);
} 
  

//��android�����ͨ�����edittext֮��Ĳ���ʹ���������
//�����ǵ�activity�е�ÿ�����ע��һ��OnTouchListener������������ֻҪ������ָ�Ӵ��������������
//�ͻᴥ��OnTouchListener��������onTouch�������Ӷ������������������̵ķ�������������̡�
public void setupUI(View view) 
{
	//Set up touch listener for non-text box views to hide keyboard.
	if(!(view instanceof EditText)) 
	{
		view.setOnTouchListener(new OnTouchListener() {
		public boolean onTouch(View v, MotionEvent event) {
			
			//�������б��������Ե���Ʒ���Ƶ��̶��ļ��С�
			saveEditTextToFile();
			hideSoftKeyboard(MainActivity.this);  //MainActivity.this���ҵ�activity��
			return false;
		}
		});
	}
	
	//If a layout container, iterate over children and seed recursion.
	if (view instanceof ViewGroup) 
	{
		for (int i = 0; i < ((ViewGroup) view).getChildCount(); i++) 
		{
			View innerView = ((ViewGroup) view).getChildAt(i);
			setupUI(innerView);
		}
	}
}  
  
  
//�������б��������Ե���Ʒ���Ƶ��̶��ļ��С�
void saveEditTextToFile()
{
	//SharedPreferencesҲ��һ�����͵����ݴ洢��ʽ�����ı����ǻ���XML�ļ��洢key-value��ֵ�����ݣ�ͨ�������洢һЩ�򵥵�������Ϣ��
	//����Context�����getSharedPreferences()������õ�SharedPreferences������Ա�ͬһӦ�ó����µ������������.
	//��洢λ����/data/data/<����>/shared_prefsĿ¼�¡�
	//SharedPreferences������ֻ�ܻ�ȡ���ݶ���֧�ִ洢���޸ģ��洢�޸���ͨ��Editor����ʵ�֡�
	//ʵ��SharedPreferences�洢�Ĳ������£� һ������Context��ȡSharedPreferences����       ��������edit()������ȡEditor����
	//	                                    ����ͨ��Editor����洢key-value��ֵ�����ݡ�    �ġ�ͨ��commit()�����ύ���ݡ�
	//Context.MODE_PRIVATEΪĬ�ϲ���ģʽ,������ļ���˽������,ֻ�ܱ�Ӧ�ñ������,�ڸ�ģʽ��,д������ݻḲ��ԭ�ļ�������
	//��δ���ִ�й��󣬼���/data/data/com.test/shared_prefsĿ¼��������һ��user.xml�ļ���һ��Ӧ�ÿ��Դ������������xml�ļ���
	
	//�����ݱ�����SharedPreferences  
	//Context.MODE_PRIVATEΪĬ�ϲ���ģʽ,������ļ���˽������,ֻ�ܱ�Ӧ�ñ������,�ڸ�ģʽ��,д������ݻḲ��ԭ�ļ�������
	SharedPreferences preferences=getSharedPreferences("user",Context.MODE_PRIVATE);
	Editor editor=preferences.edit();
	// accountString="18818275352";
	//String codeString="000000";
	
	writeNameEdit =(EditText)findViewById(R.id.nameEdit01); 
	name_ID01=writeNameEdit.getText().toString();
	
       writeNameEdit =(EditText)findViewById(R.id.nameEdit02); 
	name_ID02=writeNameEdit.getText().toString();
	
	writeNameEdit =(EditText)findViewById(R.id.nameEdit03); 
	name_ID03=writeNameEdit.getText().toString();

	editor.putString("name_ID01", name_ID01); //����//01�ű��������Ե���Ʒ����
	editor.putString("name_ID02", name_ID02); //����//02�ű��������Ե���Ʒ����
	editor.putString("name_ID03", name_ID03); //����//03�ű��������Ե���Ʒ����
	editor.commit();  // �ύ���д��������
	
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

	
////////////////////////////////////////////////////////////////
//�������(����Ʒ�������ı�������ֵ)
////////////////////////////////////////////////////////////////
void clearAlarmByHttpPost() //�������(����Ʒ�������ı�������ֵ)
{	
	//�����߳�, ���������߳�����������
	new Thread()
	{
		@Override
		public void run()
		{
			//POST������HTTP�����޸����������//alarmState0x = 01 ��ʾ���������ޱ�����
				//����: �Ʒ���ƽ̨
			post_http("http://cloud.emlab.net/iot/alarm/index.php", "alarmState01", "01", "alarmState02", "01", "alarmState03", "01");
			
				//����: �Ʒ���ƽ̨
			//post_http("http://emlab.sinaapp.com/alarm.php", "alarmState01", "01", "alarmState02", "01", "alarmState03", "01");
		}
	}.start();
}


	
////////////////////////////////////////////////////////////////
//01�ű�����:  ����"�������"��ť�Ļص�����
////////////////////////////////////////////////////////////////
public void ID01_clearAlarm_clickHandler(View source)
{
	alarmImage01.setImageResource(imageIds[6]);  //��ʾ ��ɫ��������ͼƬ
	
	//==��ʾ�������ա�===========================/
		// �ı��ı�����ı���ɫ
	ID01_timeText01.setTextColor(Color.parseColor("#000033"));  //��ɫ����
	
	//==��ʾ��ʱ���롱===========================/
		// �ı��ı�����ı���ɫ
	ID01_timeText02.setTextColor(Color.parseColor("#000033"));  //��ɫ����
}

////////////////////////////////////////////////////////////////
//02�ű�����:  ����"�������"��ť�Ļص�����
////////////////////////////////////////////////////////////////
public void ID02_clearAlarm_clickHandler(View source)
{
	alarmImage02.setImageResource(imageIds[6]);  //��ʾ ��ɫ��������ͼƬ
	
	//==��ʾ�������ա�===========================/
		// �ı��ı�����ı���ɫ
	ID02_timeText01.setTextColor(Color.parseColor("#000033"));  //��ɫ����
	
	//==��ʾ��ʱ���롱===========================/
		// �ı��ı�����ı���ɫ
	ID02_timeText02.setTextColor(Color.parseColor("#000033"));  //��ɫ����
}


////////////////////////////////////////////////////////////////
//03�ű�����:  ����"�������"��ť�Ļص�����
////////////////////////////////////////////////////////////////
public void ID03_clearAlarm_clickHandler(View source)
{
	alarmImage03.setImageResource(imageIds[6]);  //��ʾ ��ɫ��������ͼƬ
	
	//==��ʾ�������ա�===========================/
		// �ı��ı�����ı���ɫ
	ID03_timeText01.setTextColor(Color.parseColor("#000033"));  //��ɫ����
	
	//==��ʾ��ʱ���롱===========================/
		// �ı��ı�����ı���ɫ
	ID03_timeText02.setTextColor(Color.parseColor("#000033"));  //��ɫ����
}

	
	
////////////////////////////////////////////////////////////////
//����"ģ�ⱨ��"��ť�Ļص�����
////////////////////////////////////////////////////////////////
public void Alarm_clickHandler(View source)
{
	//�����߳�, ���������߳�����������
	new Thread()
	{
		@Override
		public void run()    
		{
			//POST������HTTP�����޸����������//alarmState0x = 02 ��ʾ������
				//����: �Ʒ���ƽ̨
			post_http("http://cloud.emlab.net/iot/alarm/index.php", "alarmState01", "02", "alarmState02", "02", "alarmState03", "02");
				//����: �Ʒ���ƽ̨
			//post_http("http://emlab.sinaapp.com/alarm.php", "alarmState01", "02", "alarmState02", "02", "alarmState03", "02");
			
		}
	}.start();
}


  
////////////////////////////////////////////////////////////////
//����"Test"��ť�Ļص�����
////////////////////////////////////////////////////////////////
public void Test_clickHandler(View source)
{
	//SharedPreferencesҲ��һ�����͵����ݴ洢��ʽ�����ı����ǻ���XML�ļ��洢key-value��ֵ�����ݣ�ͨ�������洢һЩ�򵥵�������Ϣ��
	//����Context�����getSharedPreferences()������õ�SharedPreferences������Ա�ͬһӦ�ó����µ������������.
	//��洢λ����/data/data/<����>/shared_prefsĿ¼�¡�
	//SharedPreferences������ֻ�ܻ�ȡ���ݶ���֧�ִ洢���޸ģ��洢�޸���ͨ��Editor����ʵ�֡�
	//ʵ��SharedPreferences�洢�Ĳ������£� һ������Context��ȡSharedPreferences����       ��������edit()������ȡEditor����
	//	                                    ����ͨ��Editor����洢key-value��ֵ�����ݡ�    �ġ�ͨ��commit()�����ύ���ݡ�
	//Context.MODE_PRIVATEΪĬ�ϲ���ģʽ,������ļ���˽������,ֻ�ܱ�Ӧ�ñ������,�ڸ�ģʽ��,д������ݻḲ��ԭ�ļ�������
	//��δ���ִ�й��󣬼���/data/data/com.test/shared_prefsĿ¼��������һ��user.xml�ļ���һ��Ӧ�ÿ��Դ������������xml�ļ���
	
	//�����ݱ�����SharedPreferences  
	//Context.MODE_PRIVATEΪĬ�ϲ���ģʽ,������ļ���˽������,ֻ�ܱ�Ӧ�ñ������,�ڸ�ģʽ��,д������ݻḲ��ԭ�ļ�������
	SharedPreferences preferences=getSharedPreferences("user",Context.MODE_PRIVATE);
	Editor editor=preferences.edit();
	String accountString="18818275352";
	String codeString="000000";
	editor.putString("account", accountString);
	editor.putString("code", codeString);
	editor.commit();  // �ύ���д��������
	
}




////////////////////////////////////////////////////////////////
//��ȡ��ǰʱ��
////////////////////////////////////////////////////////////////
void getRealTime()  //��ȡ��ǰʱ��
{
	long time=System.currentTimeMillis();
	final Calendar mCalendar=Calendar.getInstance();
	mCalendar.setTimeInMillis(time);
	
	year = mCalendar.get(Calendar.YEAR);      
	month = mCalendar.get(Calendar.MONTH)+1;      
	day = mCalendar.get(Calendar.DAY_OF_MONTH);    

	hour = mCalendar.get(Calendar.HOUR_OF_DAY); 
	minute = mCalendar.get(Calendar.MINUTE);    
	sec = mCalendar.get(Calendar.SECOND); 
}


////////////////////////////////////////////////////////////////
//��ʾ�����ľ���ʱ��
////////////////////////////////////////////////////////////////
void displayAlarmRealTime(TextView AlarmTime1,TextView AlarmTime2)
{	

//==��ʾ�������ա�===========================/

		
	// �ı��ı�����ı���ɫ
	AlarmTime1.setTextColor(Color.parseColor("#FF0033"));  //��ɫ����
	// �ı��ı�����ı�����
	AlarmTime1.setText(" "+year+"��"+month+"��"+day+"�� ");
	
//==��ʾ��ʱ���롱===========================/

	// �ı��ı�����ı���ɫ
	AlarmTime2.setTextColor(Color.parseColor("#FF0033"));  //��ɫ����
	// �ı��ı�����ı�����
	AlarmTime2.setText(" "+hour+"ʱ"+minute+"��"+sec+"�� ");



}



/****************************************************************************
*������-Function:	void Deal_workstate()
*����- Description:	 "����"��ϵͳ"״̬"�µ����� (״̬��)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    ��02)    ��03)    ��04)  
*****************************************************************************/
void Deal_workstate() //"����"��ϵͳ"״̬"�µ����� (״̬��)
{
	//���������alarmState[x] =02 ��ʾ0x�ű��������б���
	//alarmState[0] ,��ʾ01�ű�����
	if(alarmState[0].equals("02") || alarmState[1].equals("02") || alarmState[2].equals("02")) 
	{
		getRealTime();  //��ȡ��ǰʱ��
		
		//����01�ű������ı�������
		if(alarmState[0].equals("02") )
		{
			alarmImage01.setImageResource(imageIds[7]);  //��ʾ ��ɫ��������ͼƬ
			displayAlarmRealTime(ID01_timeText01,ID01_timeText02); //��ʾ�����ľ���ʱ��
		}

		//����02�ű������ı�������
		if(alarmState[1].equals("02") )
		{
			alarmImage02.setImageResource(imageIds[7]);  //��ʾ ��ɫ��������ͼƬ
			displayAlarmRealTime(ID02_timeText01,ID02_timeText02); //��ʾ�����ľ���ʱ��
		}

		//����03�ű������ı�������
		if(alarmState[2].equals("02") )
		{
			alarmImage03.setImageResource(imageIds[7]);  //��ʾ ��ɫ��������ͼƬ
			displayAlarmRealTime(ID03_timeText01,ID03_timeText02); //��ʾ�����ľ���ʱ��
		}

		//��HttpGet����ȡ�õ����ݻ���//������б���������ر���״ֵ̬
		alarmState[0] = "00";   //��01�ŵı���״ֵ̬
		alarmState[1] = "00";   //��02�ŵı���״ֵ̬
		alarmState[2] = "00";   //��03�ŵı���״ֵ̬  
		clearAlarmByHttpPost(); //������б������ı���״̬(����Ʒ�������ı�������ֵ)


		vibrator.vibrate(2000);//��ָ��ʱ�� ����������long����λΪ���룬һ����Ϊ1/1000��
		
		//����������壺��һ������Ϊ�ȴ�ָ��ʱ���ʼ�𶯣���ʱ��Ϊ�ڶ�����������ߵĲ�������Ϊ�ȴ��𶯺��𶯵�ʱ��
		//�ڶ�������Ϊ�ظ�������-1Ϊ���ظ���0Ϊһֱ�� 
		//vibrator.vibrate(new long[]{100,10,100,1000}, -1);//����ָ����ģʽȥ�𶯡�
		
		// ��ʼ��MediaPlayer����׼����������
		MediaPlayer mPlayer = MediaPlayer.create(
				MainActivity.this, R.raw.ring01);
		// ����ѭ������
		mPlayer.setLooping(false);
		// ��ʼ����
		mPlayer.start();	
	}


/*******************************************************************
	switch(workstate) 
	{
		case 0x31:  //���Ʒ���õ�������workstae�� ��ֵΪ0x31="1"����ʾ���б���״��
			break;
			
		case 0x32:  //���Ʒ���õ�������workstae�� ��ֵΪ0x32="2"����ʾ���ޱ���״��
			break;
			
		case 0x33:  
			break;	
			
		default:
			break;
	}
	*******************************************************************/
}


	
////////////////////////////////////////////////////////////////
//POST������HTTP�����޸����������
////////////////////////////////////////////////////////////////
void post_http(String url,String key1,String value1,String key2,String value2,String key3,String value3)
{	
	try
	{
		HttpPost post = new HttpPost(url);//��
		
		// ������ݲ��������Ƚ϶�Ļ����ԶԴ��ݵĲ������з�װ
		List<NameValuePair> params = new ArrayList<NameValuePair>();
		params.add(new BasicNameValuePair(key1, value1));
		params.add(new BasicNameValuePair(key2, value2));
		params.add(new BasicNameValuePair(key3, value3));
								
		// �����������
		post.setEntity(new UrlEncodedFormEntity(params, HTTP.UTF_8));
		
		// ����POST����
		HttpResponse response = httpClient.execute(post);  //��//ִ��POST���� 
		
		// ����������ɹ��ط�����Ӧ
		if (response.getStatusLine().getStatusCode() == 200)
		{
			String msg = EntityUtils.toString(response.getEntity());
			
			Message msgPost = new Message();
			msgPost.what = 4;
			myHandler.sendMessage(msgPost);  //���� message 
			
			/****
			Looper.prepare();
			// ��ʾ��¼�ɹ�
			Toast.makeText(HttpClientTest.this,
				msg, Toast.LENGTH_SHORT).show();
			Looper.loop();
			**********/
		}
	}
	catch (Exception e)
	{
		e.printStackTrace();
	}
}


////////////////////////////////////////////////////////////////
//GET������HTTP������ȡ���������
////////////////////////////////////////////////////////////////
void get_http(String url)
{
	// ����һ��HttpGet����
	HttpGet get = new HttpGet(url);  //��

	try
	{
		// ����GET����
		HttpResponse httpResponse = httpClient.execute(get);//��
		HttpEntity entity = httpResponse.getEntity();
		
		//��ȡ���ص�����, ���Դ�����
		if (entity != null)
		{
			// ��ȡ��������Ӧ
			BufferedReader br = new BufferedReader(
				new InputStreamReader(entity.getContent()));
			String line = null;

			int i = 0;
			while ((line = br.readLine()) != null)
			{
				alarmState[i] = line;
				i++;
				
			}


			if(workOnlyOnce == 1)
			{
				workOnlyOnce = 0;
				alarmState[0] = "00";   //��01�ŵı���״ֵ̬
				alarmState[1] = "00";   //��02�ŵı���״ֵ̬
				alarmState[2] = "00";   //��03�ŵı���״ֵ̬  

			}
			


			Message msg = new Message();
			msg.what = 3;
			//msg.obj = stringTemp;
			myHandler.sendMessage(msg);  //���� message 



		}
	}
	catch (Exception e)
	{
		e.printStackTrace();
	}
}


		

	

//////////////////////////////////////////////////////////
//����Handler��timer��TimerTask��ϵķ�����ʵ�ֶ�ʱѭ������
//Handler����ͨ��message�ڸ����̼߳䴫��ͨ��  
final Handler myHandler = new Handler()
{
	// ���յ���Ϣ����  
	@Override
	public void handleMessage(Message msg)
	{
	
		switch (msg.what) 
		{    
			case 1:    
				//UI����  
				break;  
			
			case 2:  //ÿ1�붨ʱ������ȡһ�α���״̬//GET������HTTP������ȡ���������
				new Thread()
				{
					@Override
					public void run()
					{	
						//GET������HTTP������ȡ���������//alarmState0x ����״ֵ̬
							//����: �Ʒ���ƽ̨
						get_http("http://cloud.emlab.net/iot/alarm/index.php");
	
							//����: �Ʒ���ƽ̨	
						//get_http("http://emlab.sinaapp.com/alarm.php");
			    		}
			    	}.start();			      
				break;  		 
				
			case 3:  //Http Get ���Ʒ�������ȡ���������
				// ʹ��response�ı�����ʾ��������Ӧ
				//response.append(msg.obj.toString() + "\n");
				//response.setText("AA"+msg.obj.toString()+"BB");
				//showAlarmTime.append(msg.obj.toString());


				Deal_workstate(); //"����"��ϵͳ"״̬"�µ����� (״̬��)
                    		/*****************************
				char[] array = string1.toCharArray();  //�����յ����ַ�����ת��������		
				workstate = array[0];
				
				
				******************************/
				break;  
				
			case 4:     
				break; 
		}    
		super.handleMessage(msg); 
	}
};






	
} //End  //Activity
