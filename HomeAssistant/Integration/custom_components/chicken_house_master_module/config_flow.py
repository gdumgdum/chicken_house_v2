""" Le Config Flow """

import logging
import voluptuous as vol
from typing import Any
import copy
from collections.abc import Mapping
#import serial.tools.list_ports
import sys
import glob
import serial

from homeassistant.config_entries import ConfigFlow, OptionsFlow, ConfigEntry
from .const import *  # pylint: disable=wildcard-import, unused-wildcard-import
import homeassistant.helpers.config_validation as cv
from homeassistant.data_entry_flow import FlowResult
from homeassistant.helpers import selector
from homeassistant.core import callback

_LOGGER = logging.getLogger(__name__)

def add_suggested_values_to_schema(
    data_schema: vol.Schema, suggested_values: Mapping[str, Any]
) -> vol.Schema:
    """Make a copy of the schema, populated with suggested values.

    For each schema marker matching items in `suggested_values`,
    the `suggested_value` will be set. The existing `suggested_value` will
    be left untouched if there is no matching item.
    """
    schema = {}
    for key, val in data_schema.schema.items():
        new_key = key
        if key in suggested_values and isinstance(key, vol.Marker):
            # Copy the marker to not modify the flow schema
            new_key = copy.copy(key)
            new_key.description = {"suggested_value": suggested_values[key]}
        schema[new_key] = val
    _LOGGER.debug("add_suggested_values_to_schema: schema=%s", schema)
    return vol.Schema(schema)

def get_serial_ports():
    #ports = serial.tools.list_ports.comports()

    #for port, desc, hwid in sorted(ports):
    #    _LOGGER.debug("Detected serial port: [%s]", port)
    
    #return ports
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/serial/by-id/*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
            _LOGGER.debug("Detected serial port: [%s]", port)
        except (OSError, serial.SerialException):
            pass
    return result



class ChickenHouseMasterModuleConfigFlow(ConfigFlow, domain=DOMAIN):
    """La classe qui implémente le config flow pour notre DOMAIN.
    Elle doit dériver de FlowHandler"""

    # La version de notre configFlow. Va permettre de migrer les entités
    # vers une version plus récente en cas de changement
    VERSION = 1

    _user_inputs: dict = {}
    _configuration_form = vol.Schema(  # pylint: disable=invalid-name
    {
        vol.Required(CONF_NAME): cv.string,
        vol.Required(CONF_SERIAL_PORT): cv.string,
        vol.Required(
            CONF_SERIAL_BAUDRATE, default=CONF_SERIAL_BAUDRATE_DEFAULT
        ): selector.SelectSelector(
            selector.SelectSelectorConfig(
                options=CONF_SERIAL_BAUDRATES, mode="dropdown" #, translation_key="baudrate"
            )
        ),
        vol.Required(
            CONF_RF_CHANNEL, default=CONF_RF_CHANNEL_DEFAULT
        ): selector.SelectSelector(
            selector.SelectSelectorConfig(
                options=CONF_RF_CHANNELS, mode="list" #, translation_key="rf_channel"
            )
        ),
        vol.Required(
            CONF_RF_PALEVEL, default=CONF_RF_PALEVEL_DEFAULT
        ): selector.SelectSelector(
            selector.SelectSelectorConfig(
                options=CONF_RF_PALEVELS, mode="dropdown" #, translation_key="rf_pa_level"
            ),
        ),
        vol.Required(
            CONF_RF_DATARATE, default=CONF_RF_DATARATE_DEFAULT
        ): selector.SelectSelector(
            selector.SelectSelectorConfig(
                options=CONF_RF_DATARATES, mode="dropdown" #, translation_key="rf_data_rate"
            ),
        ),
    }
    )


    async def async_step_user(self, user_input: dict | None = None) -> FlowResult:
        """Gestion de l'étape 'user'. Point d'entrée de notre
        configFlow. Cette méthode est appelée 2 fois :
        1. une première fois sans user_input -> on affiche le formulaire de configuration
        2. une deuxième fois avec les données saisies par l'utilisateur dans user_input -> on sauvegarde les données saisies
        """
        get_serial_ports()

        if user_input is None:
            _LOGGER.debug(
                "(%d) config_flow step user (1). 1er appel : pas de user_input -> on affiche le form user_form",
                id(self._user_inputs)
            )
            return self.async_show_form(
                step_id="user",
                data_schema=add_suggested_values_to_schema(
                    data_schema=self._configuration_form, suggested_values=self._user_inputs
                ),
            )

        # 2ème appel : il y a des user_input -> on stocke le résultat
        # TODO: utiliser les user_input
        _LOGGER.debug(
            "(%d) config_flow step user (2). On a reçu les valeurs: %s", 
            id(self._user_inputs),
            user_input
        )

        self._user_inputs.update(user_input)

        return self.async_create_entry(
            title=user_input[CONF_NAME], data=user_input
        )

    @staticmethod
    @callback
    def async_get_options_flow(config_entry: ConfigEntry):
        """Get options flow for this handler"""
        return ChickenHouseMasterModuleOptionsFlow(config_entry)

class ChickenHouseMasterModuleOptionsFlow(OptionsFlow):
    """La classe qui implémente le option flow pour notre DOMAIN.
    Elle doit dériver de OptionsFlow"""

    _user_inputs: dict = {}
    # Pour mémoriser la config en cours
    config_entry: ConfigEntry = None
    _configuration_form = vol.Schema(  # pylint: disable=invalid-name
    {
        vol.Required(CONF_NAME): cv.string,
        vol.Required(CONF_SERIAL_PORT): cv.string,
        vol.Required(
            CONF_SERIAL_BAUDRATE, default=CONF_SERIAL_BAUDRATE_DEFAULT
        ): selector.SelectSelector(
            selector.SelectSelectorConfig(
                options=CONF_SERIAL_BAUDRATES, mode="dropdown" #, translation_key="baudrate"
            )
        ),
        vol.Required(
            CONF_RF_CHANNEL, default=CONF_RF_CHANNEL_DEFAULT
        ): selector.SelectSelector(
            selector.SelectSelectorConfig(
                options=CONF_RF_CHANNELS, mode="list" #, translation_key="rf_channel"
            )
        ),
        vol.Required(
            CONF_RF_PALEVEL, default=CONF_RF_PALEVEL_DEFAULT
        ): selector.SelectSelector(
            selector.SelectSelectorConfig(
                options=CONF_RF_PALEVELS, mode="dropdown" #, translation_key="rf_pa_level"
            ),
        ),
        vol.Required(
            CONF_RF_DATARATE, default=CONF_RF_DATARATE_DEFAULT
        ): selector.SelectSelector(
            selector.SelectSelectorConfig(
                options=CONF_RF_DATARATES, mode="dropdown" #, translation_key="rf_data_rate"
            ),
        ),
    }
    )

    def __init__(self, config_entry: ConfigEntry) -> None:
        """Initialisation de l'option flow. On a le ConfigEntry existant en entrée"""
        self.config_entry = config_entry
        # On initialise les user_inputs avec les données du configEntry
        self._user_inputs = config_entry.data.copy()

    async def async_step_init(self, user_input: dict | None = None) -> FlowResult:
        """Gestion de l'étape 'init'. Point d'entrée de notre
        optionsFlow. Comme pour le ConfigFlow, cette méthode est appelée 2 fois
        """

        if user_input is None:
            _LOGGER.debug(
                "option_flow step user (1). 1er appel : pas de user_input -> "
                "on affiche le form user_form"
            )
            return self.async_show_form(
                step_id="init",
                data_schema=add_suggested_values_to_schema(
                    data_schema=self._configuration_form, suggested_values=self._user_inputs
                    ),
            )

        # 2ème appel : il y a des user_input -> on stocke le résultat
        _LOGGER.debug(
            "option_flow step user (2). On a reçu les valeurs: %s", user_input
        )
        # On mémorise les user_input
        self._user_inputs.update(user_input)

        # On appelle le step de fin pour enregistrer les modifications
        return await self.async_end()

    async def async_end(self):
        """Finalization of the ConfigEntry creation"""
        _LOGGER.info(
            "Recreation de l'entry %s. La nouvelle config est maintenant : %s",
            self.config_entry.entry_id,
            self._user_inputs,
        )

        # Modification de la configEntry avec nos nouvelles valeurs
        self.hass.config_entries.async_update_entry(
            self.config_entry, data=self._user_inputs
        )
        return self.async_create_entry(title=None, data=None)
