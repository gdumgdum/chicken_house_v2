""" Implements the VersatileThermostat sensors component """
import logging

from homeassistant.core import HomeAssistant
from homeassistant.config_entries import ConfigEntry
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.components.sensor import SensorEntity

_LOGGER = logging.getLogger(__name__)


async def async_setup_platform(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
    discovery_info=None,  # pylint: disable=unused-argument
):
    """Configuration de la plate-forme tuto_hacs à partir de la configuration
    trouvée dans configuration.yaml"""

    _LOGGER.debug("Calling async_setup_entry entry=%s", entry)

    entity = ChickenHouseTemperatureEntity(hass, entry)
    async_add_entities([entity], True)


class ChickenHouseTemperatureEntity(SensorEntity):
    """La classe de l'entité TutoHacs"""
    serial_port = ""
    serial_baudrate = 115200
    serial_stopbits = 1
    serial_bytesize = 8
    serial_parity = 'N'
    channel = 111
    pa_level = "LOW"
    data_rate = "1MBPS"
    previous_attr_native_value = -99.0
 
    @property
    def icon(self) -> str | None:
        return "mdi:thermometer-lines"

    @property
    def device_class(self) -> SensorDeviceClass | None:
        return SensorDeviceClass.TEMPERATURE

    @property
    def state_class(self) -> SensorStateClass | None:
        return SensorStateClass.MEASUREMENT

    @property
    def native_unit_of_measurement(self) -> str | None:
        return UnitOfTemperature.CELSIUS

    @property
    def should_poll(self) -> bool:
        return True

    def __init__(
        self,
        hass: HomeAssistant,  # pylint: disable=unused-argument
        entry_infos,  # pylint: disable=unused-argument
    ) -> None:
        """Initisalisation de notre entité"""
        self._hass = hass
        self._attr_name = entry_infos.get("name")
        self._attr_unique_id = entry_infos.get("entity_id")
        self._attr_has_entity_name = True
        self._attr_native_value = 0

        self.serial_port = entry_infos.get("serial_port")
        self.serial_baudrate = entry_infos.get("serial_baudrate")
        self.serial_stopbits = entry_infos.get("serial_stopbits")
        self.serial_bytesize = entry_infos.get("serial_bytesize")
        self.serial_parity = entry_infos.get("serial_parity")
        self.channel = entry_infos.get("rf_channel")
        self.pa_level = entry_infos.get("rf_power_amplifier")
        self.data_rate = entry_infos.get("rf_data_rate")

        # TODO initialize the master module
        _LOGGER.debug("Using serial [%s]", self.serial_device)


    @callback
    async def async_added_to_hass(self):
        """Ce callback est appelé lorsque l'entité est ajoutée à HA """
        self.serial = serial.serial_for_url(port_url, do_not_open=True)
        ser.baudrate = config[model]["comms"]["transport"]["baudrate"]
        ser.stopbits = config[model]["comms"]["transport"]["stopbits"]
        ser.bytesize = config[model]["comms"]["transport"]["bytesize"]
        ser.parity = config[model]["comms"]["transport"]["parity"]
        ser.timeout = config[model]["comms"]["transport"]["timeout"]
        ser.write_timeout = config[model]["comms"]["transport"]["write_timeout"]
        ser.open()
       

        # Arme le timer
        timer_cancel = async_track_time_interval(
            self._hass,
            self.fetch_temperature,   # la méthode qui sera appelée toutes les secondes
            interval=timedelta(seconds=30),
        )
        # desarme le timer lors de la destruction de l'entité
        self.async_on_remove(timer_cancel)

    @callback
    async def fetch_temperature(self, _):
        """Cette méthode va être appelée toutes les 10 secondes"""
        _LOGGER.info("Appel de fetch_temperature à %s", datetime.now())

        # TODO implement the communication with master module
        self._attr_native_value += 1
        if self._attr_native_value > 25:
            self._attr_native_value = 0

        if self._attr_native_value != self.previous_attr_native_value:
            self._hass.bus.fire(
                "event_temperature_change_ChickenHouseTemperatureEntity",
                {"temperature": self._attr_native_value},
                {"previous_temperature": self.previous_attr_native_value},
            )
            self.previous_attr_native_value = self._attr_native_value

        # On sauvegarde le nouvel état
        self.async_write_ha_state()

