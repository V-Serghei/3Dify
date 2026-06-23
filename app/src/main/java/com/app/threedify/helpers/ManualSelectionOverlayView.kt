package com.app.threedify.helpers

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class ManualSelectionOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {
    private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(54, 80, 220, 210)
        style = Paint.Style.FILL
    }
    private val strokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(128, 235, 225)
        strokeWidth = 4f
        style = Paint.Style.STROKE
    }
    private val selection = RectF()
    private var startX = 0f
    private var startY = 0f

    val selectedRect: RectF?
        get() = if (selection.width() >= MIN_SELECTION_SIZE_PX &&
            selection.height() >= MIN_SELECTION_SIZE_PX
        ) {
            RectF(selection)
        } else {
            null
        }

    fun clearSelection() {
        selection.setEmpty()
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (!selection.isEmpty) {
            canvas.drawRect(selection, fillPaint)
            canvas.drawRect(selection, strokePaint)
        }
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                startX = event.x
                startY = event.y
                updateSelection(event.x, event.y)
                return true
            }
            MotionEvent.ACTION_MOVE -> {
                updateSelection(event.x, event.y)
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                updateSelection(event.x, event.y)
                return true
            }
        }
        return true
    }

    private fun updateSelection(endX: Float, endY: Float) {
        selection.set(
            min(startX, endX).coerceIn(0f, width.toFloat()),
            min(startY, endY).coerceIn(0f, height.toFloat()),
            max(startX, endX).coerceIn(0f, width.toFloat()),
            max(startY, endY).coerceIn(0f, height.toFloat())
        )
        if (abs(selection.width()) < 1f || abs(selection.height()) < 1f) {
            selection.setEmpty()
        }
        invalidate()
    }

    companion object {
        private const val MIN_SELECTION_SIZE_PX = 24f
    }
}
