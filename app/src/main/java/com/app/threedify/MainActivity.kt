package com.app.threedify

import android.content.Intent
import android.os.Bundle
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat

import org.the3deer.app.model3D.view.ModelActivity
import org.the3deer.util.android.AndroidURLStreamHandlerFactory
import java.net.URL

class MainActivity : AppCompatActivity() {

    /////////////////////////////////////////////////////////////////
    //Should exist in every class which uses activities from VIEWERAPP
    companion object {
        init {
            System.setProperty("java.protocol.handler.pkgs", "org.the3deer.util.android")
            URL.setURLStreamHandlerFactory(AndroidURLStreamHandlerFactory())
        }
    }
    /////////////////////////////////////////////////////////////////

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_main)
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }


        /////////////////////////////////////////////////////////////////
        //Call for 3D viewer window
        //prefix - "android://"
        //package - "org.andresoviedo.dddmodel2/assets/models/"
        //file - cube.obj
        val intent = Intent(this, ModelActivity::class.java)
        intent.putExtra("uri", "android://org.andresoviedo.dddmodel2/assets/models/cube.obj")
        intent.putExtra("type", "-1")
        intent.putExtra("immersiveMode", "false")
        intent.putExtra("backgroundColor", "1.0 1.0 1.0 1.0")
        startActivity(intent)
        /////////////////////////////////////////////////////////////////
    }
}