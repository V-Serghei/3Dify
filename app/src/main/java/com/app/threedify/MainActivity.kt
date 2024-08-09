package com.app.threedify

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.view.Menu
import android.widget.Switch
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.app.AppCompatDelegate
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.drawerlayout.widget.DrawerLayout
import androidx.navigation.NavController
import androidx.navigation.findNavController
import androidx.navigation.ui.AppBarConfiguration
import androidx.navigation.ui.navigateUp
import androidx.navigation.ui.setupActionBarWithNavController
import androidx.navigation.ui.setupWithNavController
import com.app.threedify.databinding.ActivityMainBinding

import org.the3deer.app.model3D.view.ModelActivity
import org.the3deer.util.android.AndroidURLStreamHandlerFactory
import java.net.URL
import com.google.android.material.navigation.NavigationView

class MainActivity : AppCompatActivity() {

    //////////////////
    ////for interface
    //////////////////
    private lateinit var appBarConfiguration: AppBarConfiguration
    private lateinit var binding: ActivityMainBinding
    private lateinit var switchTheme: Switch
    private lateinit var navController: NavController

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


        //////////////////
        ////for interface
        //////////////////
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setContentView(R.layout.activity_main)

        setSupportActionBar(binding.appBarMain.toolbar)
        navController = findNavController(R.id.nav_host_fragment_content_main_base)

        //open camera
//        binding.appBarMain.fab.setOnClickListener {
//            openCameraFragment()
//        }

        val drawerLayout: DrawerLayout = binding.drawerLayout
        val navView: NavigationView = binding.navView

        appBarConfiguration = AppBarConfiguration(
            setOf(
                R.id.nav_home, R.id.nav_gallery, R.id.nav_camera, R.id.nav_about_us, R.id.nav_settings
            ), drawerLayout
        )
        setupActionBarWithNavController(navController, appBarConfiguration)
        navView.setupWithNavController(navController)


        //theme night - light
        switchTheme = navView.findViewById(R.id.switch_theme)
        switchTheme.isChecked = getSavedThemeState()

        switchTheme.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) {
                setDarkTheme()
            } else {
                setLightTheme()
            }
            saveThemeState(isChecked)
        }
        if (switchTheme.isChecked) {
            setDarkTheme()
        } else {
            setLightTheme()
        }
// //////////////////////////////////////////////////
        ///////////////////////////////////////////////
        //////////////////////////////////////////////////
        //////////////////////////////////////////////////



        enableEdgeToEdge()
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.drawer_layout)) { v, insets ->
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



    //////////////////
    ////for interface
    //////////////////

//    private fun openCameraFragment() {
//        navController.navigate(R.id.nav_camera)
//    }
    private fun setLightTheme() {
        AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_NO)
    }

    private fun setDarkTheme() {
        AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_YES)
    }

    private fun getSavedThemeState(): Boolean {
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        return sharedPreferences.getBoolean("isDarkTheme", false)
    }

    private fun saveThemeState(isDarkTheme: Boolean) {
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putBoolean("isDarkTheme", isDarkTheme)
        editor.apply()
    }

    override fun onCreateOptionsMenu(menu: Menu): Boolean {
        menuInflater.inflate(R.menu.main, menu)

        return true
    }
    //////////////////
    ////for interface
    //////////////////
    override fun onSupportNavigateUp(): Boolean {
        val navController = findNavController(R.id.nav_host_fragment_content_main_base)
        return navController.navigateUp(appBarConfiguration) || super.onSupportNavigateUp()
    }
}