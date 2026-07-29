package com.synchronisationwand.control.ui.components

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Check
import androidx.compose.material.icons.filled.Visibility
import androidx.compose.material.icons.filled.VisibilityOff
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.text.input.VisualTransformation
import androidx.compose.ui.unit.dp

/** A read-only label/value line, used for characteristics the wand only exposes for reading. */
@Composable
fun ReadOnlyRow(label: String, value: String, modifier: Modifier = Modifier) {
    Row(
        modifier = modifier.fillMaxWidth().padding(vertical = 6.dp),
        horizontalArrangement = androidx.compose.foundation.layout.Arrangement.SpaceBetween,
    ) {
        Text(label, style = MaterialTheme.typography.bodyMedium)
        Text(
            value,
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

/**
 * A label + editable text field with a save button that only appears once the user has
 * changed the value away from what's currently known from the device, and the value passes
 * [validate].
 */
@Composable
fun EditableRow(
    label: String,
    currentValue: String,
    supportingText: String? = null,
    isPassword: Boolean = false,
    keyboardType: KeyboardType = KeyboardType.Text,
    validate: (String) -> Boolean = { true },
    onSave: (String) -> Unit,
    modifier: Modifier = Modifier,
) {
    var text by remember(currentValue) { mutableStateOf(currentValue) }
    var passwordVisible by remember { mutableStateOf(false) }

    val isDirty = text != currentValue
    val isValid = validate(text)

    Column(modifier = modifier.fillMaxWidth().padding(vertical = 6.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            OutlinedTextField(
                value = text,
                onValueChange = { text = it },
                label = { Text(label) },
                singleLine = true,
                isError = isDirty && !isValid,
                visualTransformation = if (isPassword && !passwordVisible) {
                    PasswordVisualTransformation()
                } else {
                    VisualTransformation.None
                },
                keyboardOptions = KeyboardOptions(keyboardType = keyboardType),
                trailingIcon = if (isPassword) {
                    {
                        IconButton(onClick = { passwordVisible = !passwordVisible }) {
                            Icon(
                                if (passwordVisible) Icons.Default.VisibilityOff else Icons.Default.Visibility,
                                contentDescription = if (passwordVisible) "Hide" else "Show",
                            )
                        }
                    }
                } else {
                    null
                },
                modifier = Modifier.weight(1f),
            )
            if (isDirty && isValid) {
                IconButton(onClick = { onSave(text) }) {
                    Icon(Icons.Default.Check, contentDescription = "Save $label")
                }
            }
        }
        if (supportingText != null) {
            Text(
                supportingText,
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                modifier = Modifier.padding(start = 4.dp, top = 2.dp),
            )
        }
    }
}

/** Convenience wrapper of [EditableRow] for a nullable Float value shown/entered in milliseconds. */
@Composable
fun EditableFloatRow(
    label: String,
    unit: String,
    currentValue: Float?,
    minValue: Float = 0.001f,
    supportingText: String? = null,
    onSave: (Float) -> Unit,
    modifier: Modifier = Modifier,
) {
    EditableRow(
        label = "$label ($unit)",
        currentValue = currentValue?.let { "%.3f".format(it) } ?: "",
        supportingText = supportingText,
        keyboardType = KeyboardType.Decimal,
        validate = { it.toFloatOrNull()?.let { f -> f >= minValue } == true },
        onSave = { text -> text.toFloatOrNull()?.let(onSave) },
        modifier = modifier,
    )
}

/** Convenience wrapper of [EditableRow] for a nullable Int value within [range]. */
@Composable
fun EditableIntRow(
    label: String,
    unit: String,
    currentValue: Int?,
    range: IntRange = 0..65535,
    supportingText: String? = null,
    onSave: (Int) -> Unit,
    modifier: Modifier = Modifier,
) {
    EditableRow(
        label = if (unit.isNotEmpty()) "$label ($unit)" else label,
        currentValue = currentValue?.toString() ?: "",
        supportingText = supportingText,
        keyboardType = KeyboardType.Number,
        validate = { it.toIntOrNull()?.let { i -> i in range } == true },
        onSave = { text -> text.toIntOrNull()?.let(onSave) },
        modifier = modifier,
    )
}
