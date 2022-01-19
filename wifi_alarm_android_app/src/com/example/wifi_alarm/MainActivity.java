package com.example.wifi_alarm;

import android.app.Activity;
import android.app.Service;


//注意我们一般不自己创建menu，因为每个Activity默认都自带了一个，我们要做的是为它加菜单项和响应菜单项的点击事件(两个回调方法)。
//OnCreateOptionsMenu()   用于创建菜单项
//OnOptionsMenuSelected() 用于响应菜单项的点击

import android.util.Log;
//每个activity包含一个菜单，一个菜单又能包含多个菜单项和多个子菜单，子菜单其实也是菜单（因为它实现了Menu接口），
//因此子菜单也可以包含多个菜单项。
import android.view.Menu; //android.view.Menu接口代表一个菜单，android用它来管理各种菜单项。
import android.view.MenuItem; //android.view.MenuItem代表每个菜单项  //android.view.SubMenu代表子菜单
import android.view.View.OnTouchListener;

import android.view.View;  //导入  android.view.view(android的一个封装类)，引入后就可以用view中的方法了
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


//数据存储
import android.content.Context;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;

//一个Activity是一个应用程序组件，提供一个屏幕，用户用来交互为了完成某项任务，例如拨号、拍照、发送email、看地图。
//每一个activity被给予一个窗口，在上面可以绘制用户接口。窗口通常充满屏幕，但也可以小于屏幕而浮于其它窗口之上。
//一个应用程序通常由多个activities组成，他们通常是松耦合关系。
//通常，一个应用程序中的activity被指定为"main"activity，当第一次启动应用程序的时候呈现给用户的那个activity。
//每一个activity然后可以启动另一个activity为了完成不同的动作。
//每一次一个activity启动，前一个activity就停止了，但是系统保留activity在一个栈上（“back stack”）。
//当一个新activity启动，它被推送到栈顶，取得用户焦点。Back Stack符合简单“后进先出”原则，
//所以，当用户完成当前activity然后点击back按钮，它被弹出栈（并且被摧毁），然后之前的activity恢复。

public class MainActivity extends Activity 
{
	// 定义一个访问图片的数组
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

	ImageView  alarmImage01;   //01号报警器，对应的"状态图片"
	ImageView  alarmImage02;  //02号报警器，对应的"状态图片"
	ImageView  alarmImage03;  //03号报警器，对应的"状态图片"

	int year;      
	int month;      
	int day;    

	int hour; 
	int minute;    
	int sec;


	TextView ID01_timeText01;  //01号报警器:年月日
	TextView ID01_timeText02;  //01号报警器:时分秒
	
	TextView ID02_timeText01;  //02号报警器:年月日
	TextView ID02_timeText02;  //02号报警器:时分秒
	
	TextView ID03_timeText01;  //03号报警器:年月日
	TextView ID03_timeText02;  //03号报警器:时分秒
					
	AudioManager aManager;  //铃声相关
	Vibrator vibrator;   //振动相关



	
	HttpClient httpClient;	
	String[] alarmState = new String[10];



	

	int workOnlyOnce = 1;   //只在打开APP时，使用一次
	
	int workstate = 0;      //系统状态变量  // 状态机
	//值为1时：使能报警动作，表示可以触发一次报警动作 （更新报警时间、铃声、振动提示）
	//值为0时：表示可以不进行报警动作，即不重复触发报警运作 
	//int enAlarmTimeChange = 0; 
	
	String account;
	String code;
	
	String name_ID01;	//01号报警器测试的物品名称
	String name_ID02;	//02号报警器测试的物品名称
	String name_ID03;	//03号报警器测试的物品名称
	EditText readNameEdit ;  //专门读取:用于读取"用户文件"报警器的名称
	EditText writeNameEdit ;  //专门写入:用于读取"用户文件"报警器的名称




	

		
	
	//Activity的生命周期第一步，创建。
  @Override
  protected void onCreate(Bundle savedInstanceState)
  {
  	//super就是调用父类的属性或者方法，反之this就是调用本类中的属性和方法
  	//super.onCreate(savedInstanceState)是调用父类的onCreate构造函数
  	//savedInstanceState是保存当前Activity的状态信息
  	//onCreate方法的参数是一个Bundle类型的参数
  	//Bundle类型的数据与Map类型的数据相似，都是以key-value的形式存储数据的。
	  super.onCreate(savedInstanceState);
	  // 使用activity_main.xml文件定义的界面布局
	  setContentView(R.layout.activity_main);
	  
  
	 // 定义一个计时器，让该计时器周期性地执行指定任务
	 new Timer().schedule(new TimerTask()
	 {
		@Override
		public void run()
		{
			//定时任务,定时发送message  
			Message msgTiming = new Message(); 
			msgTiming.what = 2;   
			myHandler.sendMessage(msgTiming);  //发送message 
		}
	 }, 0, 1000); //1000表示：1000ms，即定时每 1 秒执行一次 	
		
		  
	// 获取系统的音频服务
	aManager = (AudioManager) getSystemService(
		Service.AUDIO_SERVICE);


	// 获取系统的Vibrator服务
	vibrator = (Vibrator) getSystemService(
		Service.VIBRATOR_SERVICE);

	alarmImage01 = (ImageView) findViewById(R.id.imageView1);   //01号报警器，对应的"状态图片"
	alarmImage02 = (ImageView) findViewById(R.id.imageView2);  //02号报警器，对应的"状态图片"
	alarmImage03 = (ImageView) findViewById(R.id.imageView3);  //03号报警器，对应的"状态图片"


	ID01_timeText01 =  (TextView) findViewById(R.id.ID01_timeText01);  //01号报警器:年月日
	ID01_timeText02 =  (TextView) findViewById(R.id.ID01_timeText02);  //01号报警器:时分秒
	
	ID02_timeText01 =  (TextView) findViewById(R.id.ID02_timeText01);  //02号报警器:年月日
	ID02_timeText02 =  (TextView) findViewById(R.id.ID02_timeText02);  //02号报警器:时分秒
	
	ID03_timeText01 =  (TextView) findViewById(R.id.ID03_timeText01);  //03号报警器:年月日
	ID03_timeText02 =  (TextView) findViewById(R.id.ID03_timeText02);  //03号报警器:时分秒
		
	 
	 // 创建DefaultHttpClient对象
	 httpClient = new DefaultHttpClient();

	 

//SharedPreferences也是一种轻型的数据存储方式，它的本质是基于XML文件存储key-value键值对数据，通常用来存储一些简单的配置信息。
//调用Context对象的getSharedPreferences()方法获得的SharedPreferences对象可以被同一应用程序下的其他组件共享.
//其存储位置在/data/data/<包名>/shared_prefs目录下。
//SharedPreferences对象本身只能获取数据而不支持存储和修改，存储修改是通过Editor对象实现。
//实现SharedPreferences存储的步骤如下： 一、根据Context获取SharedPreferences对象       二、利用edit()方法获取Editor对象。
//                                  三、通过Editor对象存储key-value键值对数据。    四、通过commit()方法提交数据。
//Context.MODE_PRIVATE为默认操作模式,代表该文件是私有数据,只能被应用本身访问,在该模式下,写入的内容会覆盖原文件的内容
//这段代码执行过后，即在/data/data/com.test/shared_prefs目录下生成了一个user.xml文件，一个应用可以创建多个这样的xml文件。
	 
//从SharedPreferences获取数据: 
SharedPreferences preferences=getSharedPreferences("user", Context.MODE_PRIVATE);
//如果account不存在，则返回值为"0"  //account=preferences.getString("account", "none"); 如果account不存在，则返回值为"none" 
account=preferences.getString("account", "0");
code=preferences.getString("code", "0");


name_ID01 = preferences.getString("name_ID01", "0");  //01号报警器测试的物品名称
name_ID02 = preferences.getString("name_ID02", "0");  //02号报警器测试的物品名称
name_ID03 = preferences.getString("name_ID03", "0");  //03号报警器测试的物品名称


readNameEdit = (EditText) findViewById(R.id.nameEdit01); //显示//01号报警器测试的物品名称
readNameEdit.setText(name_ID01);

readNameEdit= (EditText) findViewById(R.id.nameEdit02);//显示//02号报警器测试的物品名称
readNameEdit.setText(name_ID02);

readNameEdit= (EditText) findViewById(R.id.nameEdit03);//显示//03号报警器测试的物品名称
readNameEdit.setText(name_ID03);


	//在android中如何通过点击edittext之外的部分,使软键盘隐藏
	setupUI(findViewById(R.id.mWindow));
  }

  
//Android的activity已经为我们提前创建好了android.view.Menu对象
//你只需要重写父类的onCreateOptionMenu（）方法和onOptionsItemSelected（）即可。
//并提供了回调方法onCreateOptionsMenu(Menu menu)供我们初始化菜单的内容。
//该方法只会在选项菜单第一次显示的时候被执行，
//如果你需要动态改变选项菜单的内容，请使用 onPrepareOptionsMenu(Menu)。
  
  //注意我们一般不自己创建menu，因为每个Activity默认都自带了一个，我们要做的是为它加菜单项和响应菜单项的点击事件(两个回调方法)。
  //OnCreateOptionsMenu()   用于创建菜单项
  // 此方法用于初始化菜单，其中menu参数就是即将要显示的Menu实例。返回true则显示该menu,false 则不显示;
  //(只会在第一次初始化菜单时调用)   
  @Override
  public boolean onCreateOptionsMenu(Menu menu) 
  {
      // Inflate the menu; this adds items to the action bar if it is present.
  	//在Activity类中的函数getMenuInflater()用来返回这个Activity的MenuInflater，
  	//并通过MenuInflater对象来设置menu XML里的menu作为该Activity的菜单。
      getMenuInflater().inflate(R.menu.main, menu);
      return true;
  }

  //OnOptionsMenuSelected() 用于响应菜单项的点击
  //菜单项被点击时调用，也就是菜单项的监听方法。
  @Override
  public boolean onOptionsItemSelected(MenuItem item) 
  {
      // Handle action bar item clicks here. The action bar will
      // automatically handle clicks on the Home/Up button, so long
      // as you specify a parent activity in AndroidManifest.xml.
      int id = item.getItemId();
      
      //响应每个菜单项(通过菜单项的ID)
      if (id == R.id.action_settings) {
      	//返回true表示处理完菜单项的事件，不需要将该事件继续传播下去了
          return true;
      }
      
      //对没有处理的事件，交给父类来处理
      return super.onOptionsItemSelected(item);
  }
   
  
  
  
  
//先定义一个隐藏软键盘的工具类方法
public static void hideSoftKeyboard(Activity activity) 
{
	InputMethodManager inputMethodManager = (InputMethodManager)  activity.getSystemService(Activity.INPUT_METHOD_SERVICE);
	inputMethodManager.hideSoftInputFromWindow(activity.getCurrentFocus().getWindowToken(), 0);
} 
  

//在android中如何通过点击edittext之外的部分使软键盘隐藏
//给我们的activity中的每个组件注册一个OnTouchListener监听器，这样只要我们手指接触到了其他组件，
//就会触发OnTouchListener监听器的onTouch方法，从而调用上面的隐藏软键盘的方法来隐藏软键盘。
public void setupUI(View view) 
{
	//Set up touch listener for non-text box views to hide keyboard.
	if(!(view instanceof EditText)) 
	{
		view.setOnTouchListener(new OnTouchListener() {
		public boolean onTouch(View v, MotionEvent event) {
			
			//保存所有报警器测试的物品名称到固定文件中。
			saveEditTextToFile();
			hideSoftKeyboard(MainActivity.this);  //MainActivity.this是我的activity名
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
  
  
//保存所有报警器测试的物品名称到固定文件中。
void saveEditTextToFile()
{
	//SharedPreferences也是一种轻型的数据存储方式，它的本质是基于XML文件存储key-value键值对数据，通常用来存储一些简单的配置信息。
	//调用Context对象的getSharedPreferences()方法获得的SharedPreferences对象可以被同一应用程序下的其他组件共享.
	//其存储位置在/data/data/<包名>/shared_prefs目录下。
	//SharedPreferences对象本身只能获取数据而不支持存储和修改，存储修改是通过Editor对象实现。
	//实现SharedPreferences存储的步骤如下： 一、根据Context获取SharedPreferences对象       二、利用edit()方法获取Editor对象。
	//	                                    三、通过Editor对象存储key-value键值对数据。    四、通过commit()方法提交数据。
	//Context.MODE_PRIVATE为默认操作模式,代表该文件是私有数据,只能被应用本身访问,在该模式下,写入的内容会覆盖原文件的内容
	//这段代码执行过后，即在/data/data/com.test/shared_prefs目录下生成了一个user.xml文件，一个应用可以创建多个这样的xml文件。
	
	//将数据保存至SharedPreferences  
	//Context.MODE_PRIVATE为默认操作模式,代表该文件是私有数据,只能被应用本身访问,在该模式下,写入的内容会覆盖原文件的内容
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

	editor.putString("name_ID01", name_ID01); //保存//01号报警器测试的物品名称
	editor.putString("name_ID02", name_ID02); //保存//02号报警器测试的物品名称
	editor.putString("name_ID03", name_ID03); //保存//03号报警器测试的物品名称
	editor.commit();  // 提交所有存入的数据
	
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

	
////////////////////////////////////////////////////////////////
//清除报警(清除云服务器里的报警变量值)
////////////////////////////////////////////////////////////////
void clearAlarmByHttpPost() //清除报警(清除云服务器里的报警变量值)
{	
	//另起线程, 不能在主线程里请求网络
	new Thread()
	{
		@Override
		public void run()
		{
			//POST操作（HTTP）：修改云里的数据//alarmState0x = 01 表示：正常（无报警）
				//阿里: 云服务平台
			post_http("http://cloud.emlab.net/iot/alarm/index.php", "alarmState01", "01", "alarmState02", "01", "alarmState03", "01");
			
				//新浪: 云服务平台
			//post_http("http://emlab.sinaapp.com/alarm.php", "alarmState01", "01", "alarmState02", "01", "alarmState03", "01");
		}
	}.start();
}


	
////////////////////////////////////////////////////////////////
//01号报警器:  单击"清除报警"按钮的回调函数
////////////////////////////////////////////////////////////////
public void ID01_clearAlarm_clickHandler(View source)
{
	alarmImage01.setImageResource(imageIds[6]);  //显示 绿色“正常”图片
	
	//==显示“年月日”===========================/
		// 改变文本框的文本颜色
	ID01_timeText01.setTextColor(Color.parseColor("#000033"));  //黑色文字
	
	//==显示“时分秒”===========================/
		// 改变文本框的文本颜色
	ID01_timeText02.setTextColor(Color.parseColor("#000033"));  //黑色文字
}

////////////////////////////////////////////////////////////////
//02号报警器:  单击"清除报警"按钮的回调函数
////////////////////////////////////////////////////////////////
public void ID02_clearAlarm_clickHandler(View source)
{
	alarmImage02.setImageResource(imageIds[6]);  //显示 绿色“正常”图片
	
	//==显示“年月日”===========================/
		// 改变文本框的文本颜色
	ID02_timeText01.setTextColor(Color.parseColor("#000033"));  //黑色文字
	
	//==显示“时分秒”===========================/
		// 改变文本框的文本颜色
	ID02_timeText02.setTextColor(Color.parseColor("#000033"));  //黑色文字
}


////////////////////////////////////////////////////////////////
//03号报警器:  单击"清除报警"按钮的回调函数
////////////////////////////////////////////////////////////////
public void ID03_clearAlarm_clickHandler(View source)
{
	alarmImage03.setImageResource(imageIds[6]);  //显示 绿色“正常”图片
	
	//==显示“年月日”===========================/
		// 改变文本框的文本颜色
	ID03_timeText01.setTextColor(Color.parseColor("#000033"));  //黑色文字
	
	//==显示“时分秒”===========================/
		// 改变文本框的文本颜色
	ID03_timeText02.setTextColor(Color.parseColor("#000033"));  //黑色文字
}

	
	
////////////////////////////////////////////////////////////////
//单击"模拟报警"按钮的回调函数
////////////////////////////////////////////////////////////////
public void Alarm_clickHandler(View source)
{
	//另起线程, 不能在主线程里请求网络
	new Thread()
	{
		@Override
		public void run()    
		{
			//POST操作（HTTP）：修改云里的数据//alarmState0x = 02 表示：报警
				//阿里: 云服务平台
			post_http("http://cloud.emlab.net/iot/alarm/index.php", "alarmState01", "02", "alarmState02", "02", "alarmState03", "02");
				//新浪: 云服务平台
			//post_http("http://emlab.sinaapp.com/alarm.php", "alarmState01", "02", "alarmState02", "02", "alarmState03", "02");
			
		}
	}.start();
}


  
////////////////////////////////////////////////////////////////
//单击"Test"按钮的回调函数
////////////////////////////////////////////////////////////////
public void Test_clickHandler(View source)
{
	//SharedPreferences也是一种轻型的数据存储方式，它的本质是基于XML文件存储key-value键值对数据，通常用来存储一些简单的配置信息。
	//调用Context对象的getSharedPreferences()方法获得的SharedPreferences对象可以被同一应用程序下的其他组件共享.
	//其存储位置在/data/data/<包名>/shared_prefs目录下。
	//SharedPreferences对象本身只能获取数据而不支持存储和修改，存储修改是通过Editor对象实现。
	//实现SharedPreferences存储的步骤如下： 一、根据Context获取SharedPreferences对象       二、利用edit()方法获取Editor对象。
	//	                                    三、通过Editor对象存储key-value键值对数据。    四、通过commit()方法提交数据。
	//Context.MODE_PRIVATE为默认操作模式,代表该文件是私有数据,只能被应用本身访问,在该模式下,写入的内容会覆盖原文件的内容
	//这段代码执行过后，即在/data/data/com.test/shared_prefs目录下生成了一个user.xml文件，一个应用可以创建多个这样的xml文件。
	
	//将数据保存至SharedPreferences  
	//Context.MODE_PRIVATE为默认操作模式,代表该文件是私有数据,只能被应用本身访问,在该模式下,写入的内容会覆盖原文件的内容
	SharedPreferences preferences=getSharedPreferences("user",Context.MODE_PRIVATE);
	Editor editor=preferences.edit();
	String accountString="18818275352";
	String codeString="000000";
	editor.putString("account", accountString);
	editor.putString("code", codeString);
	editor.commit();  // 提交所有存入的数据
	
}




////////////////////////////////////////////////////////////////
//获取当前时间
////////////////////////////////////////////////////////////////
void getRealTime()  //获取当前时间
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
//显示报警的具体时间
////////////////////////////////////////////////////////////////
void displayAlarmRealTime(TextView AlarmTime1,TextView AlarmTime2)
{	

//==显示“年月日”===========================/

		
	// 改变文本框的文本颜色
	AlarmTime1.setTextColor(Color.parseColor("#FF0033"));  //红色文字
	// 改变文本框的文本内容
	AlarmTime1.setText(" "+year+"年"+month+"月"+day+"日 ");
	
//==显示“时分秒”===========================/

	// 改变文本框的文本颜色
	AlarmTime2.setTextColor(Color.parseColor("#FF0033"));  //红色文字
	// 改变文本框的文本内容
	AlarmTime2.setText(" "+hour+"时"+minute+"分"+sec+"秒 ");



}



/****************************************************************************
*函数名-Function:	void Deal_workstate()
*描述- Description:	 "处理"各系统"状态"下的事务 (状态机)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void Deal_workstate() //"处理"各系统"状态"下的事务 (状态机)
{
	//云里的数据alarmState[x] =02 表示0x号报警器，有报警
	//alarmState[0] ,表示01号报警器
	if(alarmState[0].equals("02") || alarmState[1].equals("02") || alarmState[2].equals("02")) 
	{
		getRealTime();  //获取当前时间
		
		//处理01号报警器的报警操作
		if(alarmState[0].equals("02") )
		{
			alarmImage01.setImageResource(imageIds[7]);  //显示 红色“报警”图片
			displayAlarmRealTime(ID01_timeText01,ID01_timeText02); //显示报警的具体时间
		}

		//处理02号报警器的报警操作
		if(alarmState[1].equals("02") )
		{
			alarmImage02.setImageResource(imageIds[7]);  //显示 红色“报警”图片
			displayAlarmRealTime(ID02_timeText01,ID02_timeText02); //显示报警的具体时间
		}

		//处理03号报警器的报警操作
		if(alarmState[2].equals("02") )
		{
			alarmImage03.setImageResource(imageIds[7]);  //显示 红色“报警”图片
			displayAlarmRealTime(ID03_timeText01,ID03_timeText02); //显示报警的具体时间
		}

		//清HttpGet从云取得的数据缓存//清除所有报警器的相关报警状态值
		alarmState[0] = "00";   //清01号的报警状态值
		alarmState[1] = "00";   //清02号的报警状态值
		alarmState[2] = "00";   //清03号的报警状态值  
		clearAlarmByHttpPost(); //清除所有报警器的报警状态(清除云服务器里的报警变量值)


		vibrator.vibrate(2000);//震动指定时间 ，数据类型long，单位为毫秒，一毫秒为1/1000秒
		
		//数组参数意义：第一个参数为等待指定时间后开始震动，震动时间为第二个参数。后边的参数依次为等待震动和震动的时间
		//第二个参数为重复次数，-1为不重复，0为一直震动 
		//vibrator.vibrate(new long[]{100,10,100,1000}, -1);//按照指定的模式去震动。
		
		// 初始化MediaPlayer对象，准备播放音乐
		MediaPlayer mPlayer = MediaPlayer.create(
				MainActivity.this, R.raw.ring01);
		// 设置循环播放
		mPlayer.setLooping(false);
		// 开始播放
		mPlayer.start();	
	}


/*******************************************************************
	switch(workstate) 
	{
		case 0x31:  //从云服务得到变量“workstae” 的值为0x31="1"：表示，有报警状况
			break;
			
		case 0x32:  //从云服务得到变量“workstae” 的值为0x32="2"：表示，无报警状况
			break;
			
		case 0x33:  
			break;	
			
		default:
			break;
	}
	*******************************************************************/
}


	
////////////////////////////////////////////////////////////////
//POST操作（HTTP）：修改云里的数据
////////////////////////////////////////////////////////////////
void post_http(String url,String key1,String value1,String key2,String value2,String key3,String value3)
{	
	try
	{
		HttpPost post = new HttpPost(url);//③
		
		// 如果传递参数个数比较多的话可以对传递的参数进行封装
		List<NameValuePair> params = new ArrayList<NameValuePair>();
		params.add(new BasicNameValuePair(key1, value1));
		params.add(new BasicNameValuePair(key2, value2));
		params.add(new BasicNameValuePair(key3, value3));
								
		// 设置请求参数
		post.setEntity(new UrlEncodedFormEntity(params, HTTP.UTF_8));
		
		// 发送POST请求
		HttpResponse response = httpClient.execute(post);  //④//执行POST方法 
		
		// 如果服务器成功地返回响应
		if (response.getStatusLine().getStatusCode() == 200)
		{
			String msg = EntityUtils.toString(response.getEntity());
			
			Message msgPost = new Message();
			msgPost.what = 4;
			myHandler.sendMessage(msgPost);  //发送 message 
			
			/****
			Looper.prepare();
			// 提示登录成功
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
//GET操作（HTTP）：读取云里的数据
////////////////////////////////////////////////////////////////
void get_http(String url)
{
	// 创建一个HttpGet对象
	HttpGet get = new HttpGet(url);  //①

	try
	{
		// 发送GET请求
		HttpResponse httpResponse = httpClient.execute(get);//②
		HttpEntity entity = httpResponse.getEntity();
		
		//获取返回的流量, 可以处理了
		if (entity != null)
		{
			// 读取服务器响应
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
				alarmState[0] = "00";   //清01号的报警状态值
				alarmState[1] = "00";   //清02号的报警状态值
				alarmState[2] = "00";   //清03号的报警状态值  

			}
			


			Message msg = new Message();
			msg.what = 3;
			//msg.obj = stringTemp;
			myHandler.sendMessage(msg);  //发送 message 



		}
	}
	catch (Exception e)
	{
		e.printStackTrace();
	}
}


		

	

//////////////////////////////////////////////////////////
//采用Handler与timer及TimerTask结合的方法。实现定时循环任务
//Handler可以通过message在各个线程间传递通信  
final Handler myHandler = new Handler()
{
	// 接收到消息后处理  
	@Override
	public void handleMessage(Message msg)
	{
	
		switch (msg.what) 
		{    
			case 1:    
				//UI操作  
				break;  
			
			case 2:  //每1秒定时器，读取一次报警状态//GET操作（HTTP）：读取云里的数据
				new Thread()
				{
					@Override
					public void run()
					{	
						//GET操作（HTTP）：读取云里的数据//alarmState0x 报警状态值
							//阿里: 云服务平台
						get_http("http://cloud.emlab.net/iot/alarm/index.php");
	
							//新浪: 云服务平台	
						//get_http("http://emlab.sinaapp.com/alarm.php");
			    		}
			    	}.start();			      
				break;  		 
				
			case 3:  //Http Get 从云服务器获取到相关数据
				// 使用response文本框显示服务器响应
				//response.append(msg.obj.toString() + "\n");
				//response.setText("AA"+msg.obj.toString()+"BB");
				//showAlarmTime.append(msg.obj.toString());


				Deal_workstate(); //"处理"各系统"状态"下的事务 (状态机)
                    		/*****************************
				char[] array = string1.toCharArray();  //将接收到的字符串，转换成数据		
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
