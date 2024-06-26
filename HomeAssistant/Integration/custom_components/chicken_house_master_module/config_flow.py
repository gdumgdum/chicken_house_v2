""" Le Config Flow """

import logging
import voluptuous as vol

from homeassistant.config_entries import ConfigFlow
from .const import *  # pylint: disable=wildcard-import, unused-wildcard-import
import homeassistant.helpers.config_validation as cv
from homeassistant.data_entry_flow import FlowResult
from homeassistant.helpers import selector

_LOGGER = logging.getLogger(__name__)


class ChickenHouseMasterModuleConfigFlow(ConfigFlow, domain=DOMAIN):
    """La classe qui implémente le config flow pour notre DOMAIN.
    Elle doit dériver de FlowHandler"""

    # La version de notre configFlow. Va permettre de migrer les entités
    # vers une version plus récente en cas de changement
    VERSION = 1

    _user_inputs: dict = {}

    async def async_step_user(self, user_input: dict | None = None) -> FlowResult:
        """Gestion de l'étape 'user'. Point d'entrée de notre
        configFlow. Cette méthode est appelée 2 fois :
        1. une première fois sans user_input -> on affiche le formulaire de configuration
        2. une deuxième fois avec les données saisies par l'utilisateur dans user_input -> on sauvegarde les données saisies
        """

        configuration_form = vol.Schema(  # pylint: disable=invalid-name
        {
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

        if user_input is None:
            _LOGGER.debug(
                "config_flow step user (1). 1er appel : pas de user_input -> on affiche le form user_form"
            )
            return self.async_show_form(step_id="user", data_schema=configuration_form)

        # 2ème appel : il y a des user_input -> on stocke le résultat
        # TODO: utiliser les user_input
        _LOGGER.debug(
            "config_flow step user (2). On a reçu les valeurs: %s", user_input
        )

        self._user_inputs.update(user_input)
        